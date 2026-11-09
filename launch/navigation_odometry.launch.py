"""Point-LIO + GO2 body IMU; no mapper/localizer, RViz or accumulated PCD."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('point_lio'), 'launch',
                'mapping_unilidar_l2.launch.py'])),
            launch_arguments={
                'rviz': 'false', 'navigation_output': 'true',
                'save_pcd': 'false', 'world_frame': 'lidar', 'body_frame': 'body',
            }.items()),
    ])
