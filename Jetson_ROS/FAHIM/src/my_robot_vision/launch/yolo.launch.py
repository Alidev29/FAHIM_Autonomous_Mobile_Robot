"""
FAHIM — YOLOv11 vision node launch.

Starts yolo_node which:
  - Subscribes to /camera/color/image_raw + aligned depth + camera_info
  - Publishes /camera/yolo/image_annotated  (annotated BGR image)
  - Publishes /camera/yolo/detections       (Detection2DArray)
  - Publishes /camera/yolo/detections_3d    (Detection3DArray, in camera_color_optical_frame)
  - Publishes /camera/yolo/markers          (MarkerArray — sphere + text per detection)

Prerequisite (run once on Jetson):
  pip3 install ultralytics --break-system-packages

Launch args:
  model            (default: yolo11n.pt)  — any Ultralytics model name or .pt path
  conf_threshold   (default: 0.40)
  iou_threshold    (default: 0.45)
  device           (default: '')          — '' = auto, 'cuda:0', 'cpu'
  process_every_n  (default: 2)           — process 1 in N frames (reduces GPU load)
  max_depth_m      (default: 5.0)         — ignore detections beyond this range
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('model',           default_value='yolo11n.pt'),
        DeclareLaunchArgument('conf_threshold',  default_value='0.40'),
        DeclareLaunchArgument('iou_threshold',   default_value='0.45'),
        DeclareLaunchArgument('device',          default_value=''),
        DeclareLaunchArgument('process_every_n', default_value='2'),
        DeclareLaunchArgument('max_depth_m',     default_value='5.0'),

        Node(
            package='my_robot_vision',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model':           LaunchConfiguration('model'),
                'conf_threshold':  LaunchConfiguration('conf_threshold'),
                'iou_threshold':   LaunchConfiguration('iou_threshold'),
                'device':          LaunchConfiguration('device'),
                'process_every_n': LaunchConfiguration('process_every_n'),
                'max_depth_m':     LaunchConfiguration('max_depth_m'),
            }],
        ),
    ])
