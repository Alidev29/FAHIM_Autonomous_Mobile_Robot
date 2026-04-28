"""
FAHIM — SLAM launch (RTAB-Map, 2-D lidar + RGB-D appearance).

Assumes the hardware stack is already running:
    ros2 launch my_robot_bringup robot.launch.py
in another terminal (that one brings up rsp, arduino_bridge, imu, ekf,
rplidar, realsense).

This launch file just adds the RTAB-Map SLAM node (and optionally rtabmap_viz).

Launch args:
  use_sim_time    (default: false)
  use_rtabmap_viz (default: false)  — set true to show RTAB-Map's own GUI
  localization    (default: false)  — true = use existing map, no new learning
  database_path   (default: ~/.ros/rtabmap.db)
  delete_db       (default: false)  — true = wipe database on start (debug only)

Usage examples:
  # Normal SLAM (build / update map, database is preserved across runs):
  ros2 launch my_robot_bringup slam.launch.py

  # Localization only (robot knows the map, no new nodes added):
  ros2 launch my_robot_bringup slam.launch.py localization:=true

  # Fresh SLAM session — deletes the previous database first:
  ros2 launch my_robot_bringup slam.launch.py delete_db:=true

  # Save the current map to disk (run in a separate terminal while SLAM is active):
  ros2 run nav2_map_server map_saver_cli -f ~/fahim_map --ros-args -p map_subscribe_transient_local:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---- OpaqueFunction: evaluated at launch time with full Python context ----
# This avoids the ROS2 type-inference bug where PythonExpression('true'/'false')
# gets silently converted to a Python bool, clashing with RTAB-Map's string-typed
# Mem/* parameters that were already declared via the YAML.
def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    rtabmap_cfg = os.path.join(pkg_bringup, 'config', 'rtabmap.yaml')

    # Resolve all launch arguments to plain Python strings here.
    use_sim_time  = LaunchConfiguration('use_sim_time')
    use_viz       = LaunchConfiguration('use_rtabmap_viz')
    database_path = LaunchConfiguration('database_path')

    is_localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    do_delete_db    = LaunchConfiguration('delete_db').perform(context).lower() == 'true'

    # Topic remaps — RTAB-Map expects these exact input names.
    remappings = [
        ('rgb/image',        '/camera/color/image_raw'),
        ('rgb/camera_info',  '/camera/color/camera_info'),
        ('depth/image',      '/camera/aligned_depth_to_color/image_raw'),
        ('scan',             '/scan'),
        ('odom',             '/odometry/filtered'),
    ]

    # SLAM mode       (localization=false):
    #   Mem/IncrementalMemory  = true   → add new nodes to the map
    #   Mem/InitWMWithAllNodes = false  → normal working-memory management
    #
    # Localization mode (localization=true):
    #   Mem/IncrementalMemory  = false  → read-only map, no new nodes
    #   Mem/InitWMWithAllNodes = true   → load ALL map nodes so loop-closure
    #                                      can match anywhere in the known map
    incremental_memory = 'false' if is_localization else 'true'
    init_wm_with_all   = 'true'  if is_localization else 'false'

    # Pass --delete_db_on_start only when explicitly requested.
    rtabmap_args = ['--delete_db_on_start'] if do_delete_db else []

    # ----------------- RTAB-Map SLAM node -----------------
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_cfg,
            {
                'use_sim_time':           use_sim_time,
                'database_path':          database_path,
                # Plain Python strings — no type-inference ambiguity.
                'Mem/IncrementalMemory':  incremental_memory,
                'Mem/InitWMWithAllNodes': init_wm_with_all,
            },
        ],
        remappings=remappings,
        arguments=rtabmap_args,
    )

    # ----------------- RTAB-Map viewer (optional) -----------------
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            rtabmap_cfg,
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings,
        condition=IfCondition(use_viz),
    )

    return [rtabmap, rtabmap_viz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',    default_value='false'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('localization',    default_value='false'),
        DeclareLaunchArgument('delete_db',       default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
