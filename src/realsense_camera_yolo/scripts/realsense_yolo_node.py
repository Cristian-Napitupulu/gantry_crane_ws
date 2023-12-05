#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from ultralytics import YOLO
from realsense_camera_yolo_interfaces.srv import RealsenseYOLO

import time

class RealSenseYOLOServer(Node):
    def __init__(self):
        super().__init__("realsense_yolo_node")

        # Declare and get parameters
        self.declare_parameter(
            "model_path",
            "/home/icodes/Documents/gantry_crane_ws/src/realsense_camera_yolo/scripts/yolo_model.pt",
        )
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        # Initialize YOLO model
        self.model = YOLO(model_path)

        self.get_logger().info(f"YOLO model loaded from {model_path}")

        # Initialize YOLO action server
        self.yolo_server = self.create_service(
            RealsenseYOLO, "realsense_yolo", self.execute_yolo_callback
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Print something for debugging
        self.get_logger().info("Service server ready. Waiting for requests...")

    def execute_yolo_callback(self, request, result):
        # Convert ROS2 Image message to OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(
            request.depth_image, desired_encoding="passthrough"
        )

        # Convert depth image to colorized depth image
        colorized_depth_image = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        # Start timer
        start = time.time()
        # Extract bounding box coordinates from YOLO model results
        results = self.model(
            source=colorized_depth_image, show=False, conf=0.4, save=False
        )
        # Stop timer
        end = time.time()
        execution_time = (end - start) * 1000

        if (results and results[0].boxes):
            # Get bounding box coordinates
            bounding_box = np.round(results[0].boxes.xyxy[0].tolist()).astype(int)

            # Check if log level is set to debug before showing the image
            # if (
            #     self.get_logger().get_effective_level()
            #     == rclpy.logging.LoggingSeverity.DEBUG
            # ):
            # Draw bounding box on colorized depth image
            imgres = colorized_depth_image
            color = (255, 255, 255)
            thickness = 2
            cv2.rectangle(
                imgres,
                (bounding_box[0], bounding_box[1]),
                (bounding_box[2], bounding_box[3]),
                color,
                thickness,
            )
            # Show changed perspective colorized depth image
            cv2.imshow("Image", imgres)
            cv2.waitKey(1)

            # Return result
            result.x1, result.y1, result.x2, result.y2 = map(int, bounding_box)

            # Print result for debugging
            self.get_logger().info(
                f"Bounding box: x1: {result.x1}, y1: {result.y1}, x2: {result.x2}, y2: {result.y2}. Execution time: {execution_time:.2f}ms"
            )
        else:
            # Return result
            result.x1, result.y1, result.x2, result.y2 = -1, -1, -1, -1

            # Print result for debugging
            self.get_logger().info(
                f"No bounding box found. Execution time: {execution_time:.2f}ms"
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    realsense_yolo = RealSenseYOLOServer()
    rclpy.spin(realsense_yolo)
    realsense_yolo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
