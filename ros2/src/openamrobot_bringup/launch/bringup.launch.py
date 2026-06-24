"""
Top-level OpenAMRobot bring-up — pick simulation or real with ``sim:=``.

* ``sim:=false`` (default) — real hardware: openamrobot_drivers + openamrobot_perception
  + EKF + measured static TFs, then Nav2 with ``use_sim_time:=false``. No Gazebo window;
  RViz only if ``use_rviz:=true``. Needs the real hardware and a real map.
* ``sim:=true`` — simulation: openamrobot_gazebo (Gazebo + gz_bridge + robot_state_publisher)
  publishes the same topics, then Nav2 with ``use_sim_time:=true``. Opens the Gazebo window;
  no hardware needed.

Same Nav2 stack and the SAME ``nav2_params.yaml`` in both cases — only the data source and
the clock differ.

  ros2 launch openamrobot_bringup bringup.launch.py                 # real
  ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_rviz:=true
  ros2 launch openamrobot_bringup bringup.launch.py map:=/path/to/real_map.yaml

The includes are built inside an OpaqueFunction so ``sim`` / ``map`` are RESOLVED to concrete
strings before being forwarded — passing ``LaunchConfiguration('map')`` straight into an
include silently dropped the value (it re-resolved to the child's empty default).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    nav2 = get_package_share_directory('openamrobot_nav2')
    bringup = get_package_share_directory('openamrobot_bringup')

    sim = LaunchConfiguration('sim').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    is_sim = _truthy(sim)

    def src(pkg, rel):
        return PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', rel))

    actions = []

    # --- data source: ONE of the two ---
    if is_sim:
        gazebo = get_package_share_directory('openamrobot_gazebo')
        actions.append(IncludeLaunchDescription(src(gazebo, 'gz_simulator.launch.py')))
    else:
        actions.append(IncludeLaunchDescription(src(bringup, 'real_bringup.launch.py')))
        # Goal routing (real only). bt_navigator listens on goal_pose_nav (navigation_launch
        # remaps goal_pose -> goal_pose_nav for the docking gate). Real docking is not ported
        # yet, so relay the RViz goal straight through. Replace with the dock_trigger node
        # once real AprilTag docking lands.
        actions.append(Node(
            package='topic_tools', executable='relay', name='goal_pose_relay',
            arguments=['/goal_pose', '/goal_pose_nav']))

    # --- same Nav2 stack on top of either source (concrete sim/map values) ---
    actions.append(IncludeLaunchDescription(
        src(nav2, 'localization_launch.py'),
        launch_arguments={'map': map_file, 'use_sim_time': sim}.items()))
    actions.append(IncludeLaunchDescription(
        src(nav2, 'navigation_launch.py'),
        # use_scan_filter: sim runs the laser_filters body filter; real turns it OFF because
        # openamrobot_perception already publishes /scan_filtered (avoids a duplicate publisher).
        launch_arguments={'use_sim_time': sim, 'use_scan_filter': sim}.items()))

    if _truthy(use_rviz):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(nav2, 'rviz', 'nav2_view.rviz')],
            parameters=[{'use_sim_time': is_sim}]))

    return actions


def generate_launch_description():
    nav2 = get_package_share_directory('openamrobot_nav2')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true = Gazebo simulation; false = real hardware.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Open RViz with the Nav2 view.'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(nav2, 'maps', 'my_map.yaml'),
            description='Map YAML for AMCL/localization (use a real map for sim:=false).'),
        OpaqueFunction(function=launch_setup),
    ])
