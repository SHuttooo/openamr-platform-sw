"""
Camera bring-up for the real OpenAMRobot (Sony IMX708 via camera_ros).

Starts the camera_ros node at the resolution the intrinsics were calibrated for
(1280x720). The calibration file is THIS unit's; override on another camera.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('openamrobot_perception')
    default_info = 'file://' + os.path.join(pkg, 'config', 'camera_info.yaml')
    camera_info = LaunchConfiguration('camera_info_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='camera_info_url', default_value=default_info,
            description="Camera calibration URL (this unit's intrinsics, 1280x720)."),
        Node(
            package='camera_ros', executable='camera_node', name='camera',
            parameters=[{
                'width': 1280,
                'height': 720,
                'format': 'RGB888',
                'frame_id': 'camera_optical_frame',
                'camera_info_url': camera_info,
            }],
            respawn=True, respawn_delay=3.0,
            output='screen'),
    ])
