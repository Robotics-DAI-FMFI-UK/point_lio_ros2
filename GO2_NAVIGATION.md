# GO2 Point-LIO navigation output (robot testing pending)

The original `mapping_unilidar_l2.launch.py` remains the baseline mapping launch.
Its defaults keep PCD saving enabled and navigation output disabled.

The new standalone odometry launch starts the same estimator and body-IMU
adapter, with no RViz or accumulated PCD:

```bash
source /opt/ros/foxy/setup.bash
source ~/unitree_msgs_ws/install/setup.bash
source ~/s/pointlio_ws/install/setup.bash
ros2 launch point_lio navigation_odometry.launch.py
```

It publishes `/fastlio2/body_cloud` (`PointCloud2`, XYZ/intensity) and
`/fastlio2/lio_odom` (`Odometry`, pose only) at the processed scan rate.
These are historical interface names; Point-LIO supplies the data.
Both messages have exactly the same reference timestamp. Frame `lidar` is
the continuous odometry frame and `body` is the body-IMU coordinate frame.
Neither name implies a physical rotation. The original odometry publisher
owns `lidar -> body` TF; the compatibility publisher adds no TF authority.

Point-LIO transforms points into world coordinates as individual groups are
processed. The compatibility output applies the inverse final-state pose to
those world points, expressing the entire cloud in one body reference frame.
Its timestamp is that final processed state time, not necessarily the last raw
scan point time. Groups skipped by the estimator and nonfinite world points
are excluded. The existing body-cloud publisher only applies static extrinsics
to raw points and is deliberately not used here.

The pose-only compatibility message does not provide measured twist. Its
covariance entries are conservative placeholders, not calibrated uncertainty;
PGO/localizer use its pose. The original high-rate odometry is unchanged.

`odom_only=true` must not be used: that upstream option suppresses cloud output.
The LiDAR-to-IMU extrinsics remain the previously working estimates, not newly
calibrated values. The body-IMU adapter bridges clocks, not hardware timestamps.

## Passive checks

After rebuilding and sourcing the workspace:

```bash
ros2 run point_lio check_go2_interfaces.py --baseline --seconds 10
# With navigation_odometry.launch.py instead of the baseline launch:
ros2 run point_lio check_go2_interfaces.py --seconds 10
```

The check reports observed rates, frames, fields, publisher QoS and TF edges.
For adapted output it requires exact-stamp cloud/pose pairs and matching body
frame, and checks one buffered cloud against a nearby registered world cloud.
The world-cloud comparison allows the raw scan-end/reference-time difference
(up to 30ms); inspect the chosen timestamps if that diagnostic fails.
It checks reconstructed XYZ within 2mm, skipping points omitted by the adapter.
It never commands motion and does not compare sensor time with wall-clock age.
Run during gentle manual movement and inspect wall sharpness in RViz too.
TF-edge observations alone cannot establish a unique authority: inspect the
reported publishers and ensure only one mapping/localization stack runs.

## Local validation

`test/scan_reference_test.cpp` checks inverse-pose math with translation,
a known 90-degree turn, and combined pitch/yaw round trips. It can run without
ROS/PCL (`g++ -std=c++14 -Iinclude test/scan_reference_test.cpp -o /tmp/test-scan`).
The full ROS2 Foxy compilation and live interfaces still need robot testing.
