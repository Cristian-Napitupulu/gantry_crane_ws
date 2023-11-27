#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

from realsense_camera_yolo_interfaces.srv import RealsenseYOLO


class Realsense_YOLO(Node):
    def __init__(self):
        super().__init__("realsense_yolo_node")

        # Declare parameters
        self.declare_parameter(
            "model_path",
            "/home/cristian/Documents/gantry_crane_ws/src/realsense_camera_yolo/scripts/yolo_model.pt",
        )

        # Get parameters
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        # Initialize YOLO model
        self.model = YOLO(model_path)

        # Print something for debugging
        self.get_logger().info("YOLO model loaded from {}".format(model_path))

        # Initialize YOLO action server
        self.YOLO_server = self.create_service(
            RealsenseYOLO, "realsense_yolo", self.execute_YOLO_callback
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Print something for debugging
        self.get_logger().info("Service server ready. Waiting for requests...")

    def execute_YOLO_callback(self, request, result):
        # Convert ROS2 Image message to OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(
            request.depth_image, desired_encoding="passthrough"
        )

        # Print debug message
        self.get_logger().debug("Image received")

        # Convert depth image to colorized depth image
        colorized_depth_image = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        # Extract bounding box coordinates from YOLO model results
        results = self.model(
            source=colorized_depth_image, show=False, conf=0.4, save=False
        )

        # Draw bounding box on colorized depth image
        bounding_box = results[0].boxes.xyxy[0]
        bounding_box = np.round(bounding_box.tolist()).astype(int)

        # Print bounding box coordinates
        self.get_logger().debug("Bounding box coordinates: {}".format(bounding_box))

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

        # Check if log level is set to debug before showing the image
        # if self.get_logger().get_effective_level() == rclpy.logging.LoggingSeverity.DEBUG:
            # Show changed perspective colorized depth image
        cv2.imshow("Image", imgres)
        cv2.waitKey(1)

        # Return result
        result.x1 = int(bounding_box[0])
        result.y1 = int(bounding_box[1])
        result.x2 = int(bounding_box[2])
        result.y2 = int(bounding_box[3])

        # Print something for debugging
        self.get_logger().debug("Action server succeeded")

        return result


def main(args=None):
    rclpy.init(args=args)

    realsense_yolo = Realsense_YOLO()

    rclpy.spin(realsense_yolo)

    realsense_yolo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()