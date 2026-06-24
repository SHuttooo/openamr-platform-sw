"""
Real-robot data-source bring-up for OpenAMRobot.

Composition that brings up everything the real hardware needs so the SAME Nav2 /
SLAM / docking stack runs as in simulation -- only the data source differs:

* sim  -> openamrobot_gazebo (Gazebo + gz_bridge) publishes /odom /scan /imu camera
* real -> THIS launch publishes the same topics from real hardware:
    - openamrobot_drivers   : micro-ROS agent (Teensy) + RPLIDAR (/scan)
    - openamrobot_perception: scan body filter (/scan_filtered) + camera
    - robot_localization EKF: wheels + IMU gyro-Z -> /odom + TF odom->base_link
    - measured static TFs for THIS unit (lidar mounted rotated 180 deg)

Use ``use_sim_time:=false`` downstream (real time). See openamrobot_nav2 for the
real navigation profile (nav2_params_real.yaml + real_bringup_launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _static_tf(name, parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """Build a static_transform_publisher Node (translation in m, rotation in rad)."""
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', parent, '--child-frame-id', child],
        output='screen')


def _include(pkg_share, rel):
    """Include another package's launch file by share path."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', rel)))


def generate_launch_description():
    drivers = get_package_share_directory('openamrobot_drivers')
    perception = get_package_share_directory('openamrobot_perception')
    bringup = get_package_share_directory('openamrobot_bringup')
    ekf_params = os.path.join(bringup, 'config', 'ekf.yaml')

    return LaunchDescription([
        # --- real data sources (same topics the sim publishes from Gazebo) ---
        _include(drivers, 'drivers.launch.py'),
        _include(perception, 'scan_body_filter.launch.py'),
        _include(perception, 'camera.launch.py'),
        # --- odometry fusion: wheels + IMU gyro-Z -> /odom + TF odom->base_link ---
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', '/odom')],
            output='screen'),
        # --- measured static TFs for THIS unit ---
        # lidar: 0.335 m ahead of the axle, 0.18 m up, mounted ROTATED 180 deg.
        _static_tf('base_to_lidar', 'base_link', 'lidar_link', x=0.335, z=0.18, yaw=3.14159),
        # imu: identity (only gyro-Z used, robust to the small physical tilt).
        _static_tf('base_to_imu', 'base_link', 'imu_link'),
        # base_footprint: identity (firmware odom child_frame_id = base_footprint).
        _static_tf('base_to_footprint', 'base_link', 'base_footprint'),
        # camera: 0.415 m ahead, 0.12 m up.
        _static_tf('base_to_camera', 'base_link', 'camera_link', x=0.415, z=0.12),
        # optical frame: ROS optical convention (z fwd, x right, y down).
        _static_tf('camera_to_optical', 'camera_link', 'camera_optical_frame',
                   roll=-1.5708, yaw=-1.5708),
    ])
