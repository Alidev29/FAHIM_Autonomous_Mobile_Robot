"""Launch RViz2 with the FAHIM description config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_description')
    rviz_cfg = os.path.join(pkg_desc, 'rviz', 'my_robot.rviz')

    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
