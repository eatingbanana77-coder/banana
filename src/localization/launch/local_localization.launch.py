"""
local_localization.launch.py — Local Odometry Launch

Arguments:
  use_sim_time  — Use simulation clock (default: false)
  sync_imu      — Re-stamp IMU to wall clock for bag playback (default: false)

Usage:
  ros2 launch localization local_localization.launch.py                      # 실제 로봇
  ros2 launch localization local_localization.launch.py sync_imu:=true       # bag 재생
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    pkg_share = get_package_share_directory('localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    sync_imu = LaunchConfiguration('sync_imu').perform(context).lower() == 'true'

    imu_topic = '/imu/data/synced' if sync_imu else '/imu/data'

    nodes = [
        Node(
            package='localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{'pub_tf': False, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time, 'imu0': imu_topic}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    if sync_imu:
        nodes.append(
            Node(
                package='localization',
                executable='imu_time_sync',
                name='imu_time_sync',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sync_imu', default_value='false',
                              description='Re-stamp IMU to wall clock (for bag playback)'),
        OpaqueFunction(function=_launch_setup),
    ])
