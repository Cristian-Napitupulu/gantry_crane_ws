#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from ultralytics import YOLO
from realsense_camera_yolo_interfaces.srv import RealsenseYOLO

import time

CONTAINER_RATIO = 15 / 33.5
YOLO_ONLY_MODE = True


class ResultID:
    NoResult = 0
    YOLOResult = 1
    CornerResult = 2


# Create a moving average class to smooth out the bounding box coordinates
class MovingAverage(np.ndarray):
    def __init__(self, length):
        self.length = length
        self.queue = []

    def add(self, value):
        if len(self.queue) >= self.length:
            self.queue.pop(0)
        self.queue.append(value)

    def get(self):
        return sum(self.queue) / len(self.queue)

    def clear(self):
        self.queue = []


# Create the object of the moving average class
moving_average_execution_time = MovingAverage(30)


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

        # Start timer
        self.start = time.time()

    def execute_yolo_callback(self, request, result):
        # Convert ROS2 Image message to OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(
            request.depth_image, desired_encoding="passthrough"
        )

        # Convert depth image to colorized depth image
        self.colorized_depth_image = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        self.frame_height, self._frame_width, _ = self.colorized_depth_image.shape

        # Extract bounding box coordinates from YOLO model results
        results = self.model(
            source=self.colorized_depth_image, show=False, conf=0.4, save=False
        )
        if results and results[0].boxes:
            # Get bounding box coordinates
            bounding_box = np.round(results[0].boxes.xyxy[0].tolist()).astype(int)
            imgres = self.colorized_depth_image

            bounding_box_ratio = abs(bounding_box[2] - bounding_box[0]) / abs(
                bounding_box[3] - bounding_box[1]
            )
            entire_container_visible = True
            corners_exist = False
            # Check if entire container is visible
            if (
                bounding_box[1] == 0
                or bounding_box[3] == self.frame_height
                or (abs(bounding_box_ratio - CONTAINER_RATIO) > 0.1)
            ) and (not YOLO_ONLY_MODE):
                entire_container_visible = False
                # Find midpoint of bounding box
                corners = self.find_corner_from_YOLO_result(bounding_box)

                if (
                    corners is not None
                    and len(corners) > 0
                    and not np.any(np.isnan(corners))
                ):
                    corners_exist = True
                    for corner in corners:
                        cv2.circle(
                            imgres,
                            (int(corner[0]), int(corner[1])),
                            3,
                            (0, 0, 255),
                            thickness=cv2.FILLED,
                        )

            else:
                # Draw bounding box on colorized depth image
                offset = 5
                cv2.rectangle(
                    imgres,
                    (bounding_box[0] - offset, bounding_box[1] - offset),
                    (bounding_box[2] + offset, bounding_box[3] + offset),
                    color=(255, 255, 255),
                    thickness=2,
                )
                center = (
                    int((bounding_box[0] + bounding_box[2]) / 2),
                    int((bounding_box[1] + bounding_box[3]) / 2),
                )
                cv2.circle(imgres, center, 3, (0, 165, 255), thickness=cv2.FILLED)

            cv2.imshow("Image", imgres)

            cv2.waitKey(1)

            # Stop timer
            end = time.time()
            execution_time = (end - self.start) * 1000
            self.start = time.time()

            if entire_container_visible:
                result.id = ResultID.YOLOResult
                result.x1, result.y1, result.x2, result.y2 = (
                    int(bounding_box[0]),
                    int(bounding_box[1]),
                    int(bounding_box[2]),
                    int(bounding_box[3]),
                )
                # Print result for debugging
                self.get_logger().info(
                    f"Using Bound box (pixel): ({result.x1}, {result.y1}, {result.x2}, {result.y2}). Execution time: {execution_time:.2f}ms"
                )

            elif (
                (not entire_container_visible) and (corners_exist) and len(corners) >= 2
            ):
                result.id = ResultID.CornerResult
                result.x1, result.y1, result.x2, result.y2 = (
                    int(corners[0][0]),
                    int(corners[0][1]),
                    int(corners[1][0]),
                    int(corners[1][1]),
                )
                # Print result for debugging
                self.get_logger().info(
                    f"Using Corner (pixel): ({result.x1}, {result.y1}, {result.x2}, {result.y2}). Execution time: {execution_time:.2f}ms"
                )

            elif (not entire_container_visible) and (not corners_exist):
                result.id = ResultID.YOLOResult
                result.x1, result.y1, result.x2, result.y2 = (
                    int(bounding_box[0]),
                    int(bounding_box[1]),
                    int(bounding_box[2]),
                    int(bounding_box[3]),
                )
                # Print result for debugging
                self.get_logger().info(
                    f"Bounding box (pixel): ({result.x1}, {result.y1}, {result.x2}, {result.y2}). Execution time: {execution_time:.2f}ms"
                )

            return result

        else:
            # Return result
            result.id = ResultID.NoResult
            result.x1, result.y1, result.x2, result.y2 = -999, -999, -999, -999

            # Stop timer
            end = time.time()
            execution_time = (end - self.start) * 1000
            moving_average_execution_time.add(execution_time)
            execution_time = moving_average_execution_time.get()

            # Print result for debugging
            self.get_logger().info(
                f"No bounding box found. Execution time: {execution_time:.2f}ms"
            )

            self.start = time.time()

            return result

    def find_corner_from_YOLO_result(self, bounding_box):
        doing_top_corners = False
        OFFSET = 5
        x_lower = bounding_box[0] - OFFSET
        x_upper = bounding_box[2] + OFFSET
        y_lower = bounding_box[1] - OFFSET
        y_upper = bounding_box[3] + OFFSET
        if x_lower < 0:
            x_lower = 0
        if x_upper > self._frame_width:
            x_upper = self._frame_width
        if y_lower < 0:
            y_lower = 0
        if y_upper > self.frame_height:
            y_upper = self.frame_height

        if bounding_box[1] == 0:
            doing_top_corners = True
            y_lower = (y_upper - y_lower) / 2 + y_lower
        if bounding_box[3] == self.frame_height:
            doing_top_corners = False
            y_upper = (y_upper - y_lower) / 2 + y_lower

        image = self.colorized_depth_image[
            int(y_lower) : int(y_upper), int(x_lower) : int(x_upper)
        ].copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        intensity_threshold_min = 25
        intensity_threshold_max = 100
        intensity_mask_min = gray > intensity_threshold_min
        intensity_mask_max = gray < intensity_threshold_max
        combined_mask_intensity = np.logical_and(intensity_mask_min, intensity_mask_max)
        filtered_gray = np.zeros_like(gray)
        filtered_gray[combined_mask_intensity] = gray[combined_mask_intensity]
        cv2.imshow("filtered_gray", filtered_gray)

        # # Step 1: Apply Canny edge detection
        edges = cv2.Canny(filtered_gray, 80, 100, apertureSize=3)
        if edges is None:
            return None
        cv2.imshow("edges", edges)

        # # Step 2: Use Hough Line Transform to detect lines
        lines = cv2.HoughLines(edges, 1, np.pi / 180 / 2, threshold=20)
        if lines is None:
            return None

        # Step 3: Identify intersections to find corners
        corners = []
        for line1 in lines:
            for line2 in lines:
                rho1, theta1 = line1[0]
                rho2, theta2 = line2[0]

                # Check if lines are perpendicular (theta difference is approximately 90 degrees)
                if abs(np.abs(theta1 - theta2) - np.pi / 2) < np.pi / 18:
                    # Find intersection point
                    A = np.array(
                        [
                            [np.cos(theta1), np.sin(theta1)],
                            [np.cos(theta2), np.sin(theta2)],
                        ]
                    )
                    b = np.array([rho1, rho2])
                    intersection = np.linalg.solve(A, b)

                    # Convert intersection point to integer values
                    intersection = tuple(map(int, intersection))

                    # Append the intersection point to the corners list
                    corners.append(intersection)

        corners = np.array(corners)
        if corners.size == 0:
            return None

        boundary_x = image.shape[1] / 2

        CORNER_OFFSET_X = 7
        CORNER_OFFSET_Y = 10

        if doing_top_corners:
            CORNER_OFFSET_Y = -CORNER_OFFSET_Y

        right_corner = np.array([0, 0])
        left_corner = np.array([0, 0])
        corners_ = np.empty((0, 2), dtype=np.int32)

        left_corners = corners[corners[:, 0] < boundary_x]
        if left_corners.size != 0:
            left_corners = self.remove_outliers_2d(left_corners, threshold=1)
            left_corner = np.mean(left_corners, axis=0)
            left_corner[0] += CORNER_OFFSET_X
            left_corner[1] += CORNER_OFFSET_Y

        if not (np.any(np.isnan(left_corner)) or np.any(left_corner == 0)):
            left_corner[0] += x_lower
            left_corner[1] += y_lower
            corners_ = np.vstack((corners_, left_corner))

        right_corners = corners[corners[:, 0] > boundary_x]
        # Handle case where there are no corners in the first quadrant
        if right_corners.size != 0:
            right_corners = self.remove_outliers_2d(right_corners, threshold=1)
            right_corner = np.mean(right_corners, axis=0)
            right_corner[0] -= CORNER_OFFSET_X
            right_corner[1] += CORNER_OFFSET_Y

        if not (np.any(np.isnan(right_corner)) or np.any(right_corner == 0)):
            right_corner[0] += x_lower
            right_corner[1] += y_lower
            corners_ = np.vstack((corners_, right_corner))

        return corners_

    def remove_outliers_2d(self, points, threshold=3):
        # Calculate z-scores for both dimensions
        z_scores_x = np.abs(
            (points[:, 0] - np.mean(points[:, 0])) / np.std(points[:, 0])
        )
        z_scores_y = np.abs(
            (points[:, 1] - np.mean(points[:, 1])) / np.std(points[:, 1])
        )

        # Combine z-scores to identify outliers in either dimension
        z_scores_combined = np.sqrt(z_scores_x**2 + z_scores_y**2)

        # Keep only points with z-scores below the threshold
        return points[z_scores_combined < threshold]


def main(args=None):
    rclpy.init(args=args)
    realsense_yolo = RealSenseYOLOServer()
    rclpy.spin(realsense_yolo)
    realsense_yolo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
