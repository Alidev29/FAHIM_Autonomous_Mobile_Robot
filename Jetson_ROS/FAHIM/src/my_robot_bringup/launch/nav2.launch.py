"""
FAHIM — Nav2 navigation launch.

Requires the hardware stack AND RTAB-Map localization to already be running:
    T1: ros2 launch my_robot_bringup robot.launch.py
    T2: ros2 launch my_robot_bringup slam.launch.py localization:=true
    T3: ros2 launch my_robot_bringup nav2.launch.py   ← this file

RTAB-Map (localization mode) publishes the map→odom TF and the /map topic.
Nav2 uses that TF directly — no AMCL is started here.

The velocity smoother sits between Nav2 and arduino_bridge:
    Nav2 controller → /cmd_vel_smoothed → velocity_smoother → /cmd_vel → arduino_bridge

Launch args:
  map_path        (default: ~/FAHIM/fahim_map.yaml)  — path to saved map YAML
  use_sim_time    (default: false)
  params_file     (default: <pkg>/config/nav2_params.yaml)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_bringup  = get_package_share_directory('my_robot_bringup')
    nav2_cfg     = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    default_map  = os.path.expanduser('~/FAHIM/fahim_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    map_path     = LaunchConfiguration('map_path')

    # ------------------------------------------------------------------ #
    # map_server — serves the saved static map to Nav2 costmaps.          #
    # RTAB-Map (launched separately) also publishes /map while            #
    # localizing, so both are active; the global costmap subscribes to    #
    # /map with transient-local QoS and picks up the first publisher.     #
    # ------------------------------------------------------------------ #
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time':   use_sim_time,
                'yaml_filename':  map_path,
            },
        ],
    )

    # ------------------------------------------------------------------ #
    # controller_server — DWB local planner                               #
    # Publishes to /cmd_vel_smoothed (remapped so velocity_smoother       #
    # can intercept before arduino_bridge).                               #
    # ------------------------------------------------------------------ #
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_smoothed')],
    )

    # ------------------------------------------------------------------ #
    # planner_server — NavFn global path planner                          #
    # ------------------------------------------------------------------ #
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # smoother_server — path smoother (post-processes global path)        #
    # ------------------------------------------------------------------ #
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # behavior_server — spin, backup, wait recovery behaviors             #
    # ------------------------------------------------------------------ #
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # bt_navigator — behavior-tree navigator (NavigateToPose action)      #
    # ------------------------------------------------------------------ #
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # waypoint_follower — multi-waypoint navigation                       #
    # ------------------------------------------------------------------ #
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # velocity_smoother — smooths Nav2 cmd_vel before arduino_bridge      #
    # Subscribes /cmd_vel_smoothed (from controller), publishes /cmd_vel  #
    # ------------------------------------------------------------------ #
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel',     'cmd_vel_smoothed'),   # input from controller
            ('cmd_vel_smoothed', 'cmd_vel'),        # output to arduino_bridge
        ],
    )

    # ------------------------------------------------------------------ #
    # lifecycle_manager — brings all Nav2 nodes through configure→active  #
    # ------------------------------------------------------------------ #
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',  default_value=nav2_cfg),
        DeclareLaunchArgument('map_path',     default_value=default_map),

        map_server,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
