from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    launch_description = LaunchDescription(
        [
            Node(
                package="realsense_camera_yolo",
                executable="realsense_camera_node",
                name="realsense_camera_node",
                parameters=[
                    {"publish_depth": False},  # Default: false
                    {"publish_color": False},  # Default: false
                ],
                on_exit=[
                    Shutdown(reason="realsense_camera_node exited"),
                ],
            ),
            Node(
                package="realsense_camera_yolo",
                executable="realsense_yolo_node.py",
                name="realsense_yolo_node",
                parameters=[
                    {
                        "model_path": "/home/icodes/Documents/gantry_crane_ws/src/realsense_camera_yolo/scripts/yolo_model.pt"
                    }
                ],
                on_exit=[
                    Shutdown(reason="realsense_yolo_node exited"),
                ],
            ),
        ]
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[
                    LogInfo(msg="Launch is over"),
                    Shutdown(reason="realsense_camera_node exited"),
                ]
            )
        )
    )

    return launch_description
