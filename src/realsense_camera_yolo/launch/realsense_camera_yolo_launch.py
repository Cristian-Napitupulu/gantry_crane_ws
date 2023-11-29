from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="realsense_camera_yolo",
                executable="realsense_yolo_node.py",
                name="realsense_yolo_node",
                output="screen",
                parameters=[
                    {
                        "model_path": "/home/cristian/Documents/gantry_crane_ws/src/realsense_camera_yolo/scripts/yolo_model.pt"
                    }
                ],
            ),
            Node(
                package="realsense_camera_yolo",
                executable="realsense_camera_node",
                name="realsense_camera_node",
                parameters=[
                    {"publish_depth": True}, # Default: false
                    {"publish_color": True}, # Default: false
                ],
            ),
        ]
    )
