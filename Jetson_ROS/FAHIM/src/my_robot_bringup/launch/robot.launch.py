"""
FAHIM — full hardware bringup.

Brings up:
  - robot_state_publisher (URDF)
  - joint_state_publisher (default wheel angles so TF tree is complete)
  - arduino_bridge   (cmd_vel in / odom out)
  - rplidar_node     (RPLidar A2 M12)
  - rf2o_laser_odometry  (LiDAR scan-matching odometry → /odom_rf2o)
  - realsense2_camera (D435i, depth + RGB only — no IMU)
  - ekf_node         (fuses /odom wheel-velocity + /odom_rf2o pose)

Odometry strategy
  The EKF fuses two complementary sources:
    /odom        — wheel encoders:  provides  vx  and  vyaw  (velocity)
    /odom_rf2o   — LiDAR matching:  provides  x, y, yaw    (absolute pose)
  Wheels are reliable for forward velocity; LiDAR is accurate for rotation,
  especially during U-turns where wheel slip accumulates angular error.

Launch args:
  use_sim_time (default: false)
  use_ekf      (default: true)   — if false, arduino_bridge publishes the TF
  use_lidar    (default: true)
  use_camera   (default: true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc    = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'my_robot.urdf.xacro')
    ekf_cfg    = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf      = LaunchConfiguration('use_ekf')
    use_lidar    = LaunchConfiguration('use_lidar')
    use_camera   = LaunchConfiguration('use_camera')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        ),
    }

    # ============ robot_state_publisher ============
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ============ joint_state_publisher ============
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ============ Arduino bridge ============
    wheel_params = {
        'port': '/dev/arduino',
        'wheel_radius': 0.0425,
        'wheel_separation': 0.235,
        'encoder_cpr': 660,
        'max_rpm': 333.0,
        'left_motor_scale':  0.98,
        'right_motor_scale': 1.00,
        'left_odom_scale':   1.00,
        'right_odom_scale':  1.00,
        # Minimum RPM before dead-zone compensation kicks in.
        # Raise if the robot still stalls on pure rotation; lower if it jerks.
        'min_motor_rpm':     50.0,
    }

    arduino_ekf = Node(
        package='my_robot_hardware', executable='arduino_bridge',
        name='arduino_bridge', output='screen',
        parameters=[{**wheel_params, 'publish_tf': False}],
        condition=IfCondition(use_ekf),
    )
    arduino_noekf = Node(
        package='my_robot_hardware', executable='arduino_bridge',
        name='arduino_bridge', output='screen',
        parameters=[{**wheel_params, 'publish_tf': True}],
        condition=UnlessCondition(use_ekf),
    )

    # ============ rf2o laser odometry ============
    # Computes 2-D odometry by matching consecutive LiDAR scans.
    # Far more accurate than wheel encoders for rotation — directly
    # measures geometric angular displacement, eliminating U-turn drift.
    #
    # publish_tf = False — the EKF owns the odom→base_footprint TF.
    # The EKF fuses /odom_lidar for absolute pose (x, y, yaw) alongside
    # wheel /odom for velocity (vx, vyaw).
    #
    # Built from source: cd ~/FAHIM/src &&
    #   git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
    #   cd ~/FAHIM && colcon build --packages-select rf2o_laser_odometry
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom_lidar',
            'publish_tf':           False,
            'base_frame_id':        'base_footprint',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 10.0,
            'verbose':              False,
        }],
        condition=IfCondition(use_lidar),
    )

    # ============ EKF ============
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ekf),
    )

    # ============ RPLidar ============
    rplidar = Node(
        package='rplidar_ros', executable='rplidar_node',
        name='rplidar', output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 256000,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        condition=IfCondition(use_lidar),
    )

    # ============ RealSense D435i ============
    # Depth + RGB only. IMU streams are disabled — rotation accuracy is
    # handled by rf2o_laser_odometry instead.
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        parameters=[{
            'camera_name':        'camera',
            'camera_namespace':   '',
            'enable_gyro':        False,
            'enable_accel':       False,
            'enable_depth':       True,
            'enable_color':       True,
            'depth_width':        640,
            'depth_height':       480,
            'depth_fps':          30,
            'color_width':        640,
            'color_height':       480,
            'color_fps':          30,
            'align_depth.enable': True,
            'pointcloud.enable':  True,
            'publish_tf':         False,
            'tf_publish_rate':    0.0,
        }],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ekf',      default_value='true'),
        DeclareLaunchArgument('use_lidar',    default_value='true'),
        DeclareLaunchArgument('use_camera',   default_value='true'),
        rsp,
        jsp,
        arduino_ekf,
        arduino_noekf,
        rf2o,
        ekf,
        rplidar,
        realsense,
    ])
