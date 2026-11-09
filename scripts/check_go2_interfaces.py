#!/usr/bin/env python3
"""Passive GO2 interface check. Does not command motion or restamp sensor data."""
import argparse
from collections import Counter, deque
from functools import partial
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec


def xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if any(name not in fields or fields[name].datatype != 7 for name in ('x', 'y', 'z')):
        raise ValueError('Expected FLOAT32 x/y/z fields')
    unpack = struct.Struct(('>' if msg.is_bigendian else '<') + 'f').unpack_from
    for row in range(msg.height):
        for col in range(msg.width):
            offset = row * msg.row_step + col * msg.point_step
            yield tuple(unpack(msg.data, offset + fields[name].offset)[0]
                        for name in ('x', 'y', 'z'))


def world_point(p, pose):
    q = pose.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid pose quaternion')
    x,y,z,w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    matrix = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
              (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
              (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    t = pose.position
    return tuple(sum(matrix[i][j]*p[j] for j in range(3)) + (t.x,t.y,t.z)[i]
                 for i in range(3))


def qos_summary(profile):
    def policy(name):
        value = getattr(profile, name)
        return getattr(value, 'name', str(value))
    return 'reliability={} durability={} history={} depth={}'.format(
        policy('reliability'), policy('durability'), policy('history'), profile.depth)


class Check(Node):
    def __init__(self):
        super().__init__('go2_interface_check')
        self.counts = Counter()
        self.headers = {}
        self.poses, self.clouds = {}, {}
        self.worlds = deque(maxlen=20)
        self.pairs = deque(maxlen=20)
        self.paired = self.bad_frames = 0
        self.transforms = Counter()
        self.subs = []
        for topic, msg_type in (
            ('/utlidar/cloud', PointCloud2), ('/go2/body_imu_raw', Imu),
            ('/aft_mapped_to_init', Odometry), ('/cloud_registered', PointCloud2),
            ('/fastlio2/body_cloud', PointCloud2), ('/fastlio2/lio_odom', Odometry)):
            self.subs.append(self.create_subscription(
                msg_type, topic, partial(self.receive, topic),
                qos_profile_sensor_data))
        self.subs.append(self.create_subscription(
            TFMessage, '/tf', self.tf, qos_profile_sensor_data))

    def tf(self, msg):
        for t in msg.transforms:
            self.transforms[(t.header.frame_id, t.child_frame_id)] += 1

    def receive(self, topic, msg):
        self.counts[topic] += 1
        self.headers[topic] = msg
        key = stamp(msg)
        if topic == '/cloud_registered':
            self.worlds.append(msg)
        elif topic == '/fastlio2/lio_odom':
            self.poses[key] = msg
        elif topic == '/fastlio2/body_cloud':
            self.clouds[key] = msg
        for k in set(self.poses).intersection(self.clouds):
            pose, cloud = self.poses.pop(k), self.clouds.pop(k)
            self.paired += 1
            self.bad_frames += cloud.header.frame_id != pose.child_frame_id
            self.pairs.append((pose, cloud))
        # Bound diagnostic memory even if counterpart is absent.
        for cache in (self.poses, self.clouds):
            while len(cache) > 30:
                del cache[next(iter(cache))]

    def report(self, duration, baseline):
        for topic, count in sorted(self.counts.items()):
            msg = self.headers[topic]
            extra = ''
            if isinstance(msg, PointCloud2):
                extra = ' fields=' + ','.join(f.name for f in msg.fields)
            if isinstance(msg, Odometry):
                extra = ' child=' + msg.child_frame_id
            print('{}: received={:.1f} Hz frame={} stamp_ns={}{}'.format(
                topic, count/duration, msg.header.frame_id, stamp(msg), extra))
            for ep in self.get_publishers_info_by_topic(topic):
                print('  publisher={} QoS={}'.format(
                    ep.node_namespace.rstrip('/') + '/' + ep.node_name,
                    qos_summary(ep.qos_profile)))
        print('Observed TF edges (check publisher identities separately):', dict(self.transforms))
        if baseline:
            return all(self.counts[t] for t in (
                '/utlidar/cloud', '/go2/body_imu_raw', '/cloud_registered', '/aft_mapped_to_init'))
        print('Exact-stamp pairs={} frame mismatches={}'.format(self.paired, self.bad_frames))
        if not self.pairs or not self.worlds or self.bad_frames:
            return False
        # Choose a pair with a nearby source world cloud; filtered scan-end time
        # may differ from reference time. Check ordered valid-point subsequence.
        pose, cloud, world = min(
            ((p,c,w) for p,c in self.pairs for w in self.worlds),
            key=lambda v: abs(stamp(v[0])-stamp(v[2])))
        delta = abs(stamp(pose)-stamp(world))*1e-9
        if delta > .03:
            print('No world cloud within 30ms of a paired reference; inspect source timestamps.')
            return False
        candidates = iter(xyz(world))
        matched = total = 0
        for p in xyz(cloud):
            total += 1
            predicted = world_point(p, pose.pose.pose)
            for w in candidates:
                if max(abs(a-b) for a,b in zip(predicted,w)) < .002:
                    matched += 1
                    break
        print('World reconstruction: {}/{} points within 2mm; source stamp delta={:.6f}s'.format(
            matched,total,delta))
        return total > 0 and matched == total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--baseline', action='store_true', help='Check original outputs only')
    args, ros_args = parser.parse_known_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init(args=ros_args)
    node = Check()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        ok = node.report(time.monotonic()-start, args.baseline)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
