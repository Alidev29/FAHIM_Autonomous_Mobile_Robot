import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'my_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    # xacro -> URDF string
    # Wrap in ParameterValue(str) so Humble doesn't YAML-parse the XML.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        ),
    }

    # robot_state_publisher — publishes all fixed TFs + joint TFs
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # joint_state_publisher_gui — interactive sliders for wheel joints (testing only)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    )

    # joint_state_publisher — no GUI, publishes 0 for all joints (for headless tests)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
    )

    # rviz2 — visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Show joint_state_publisher_gui for manual joint control',
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
