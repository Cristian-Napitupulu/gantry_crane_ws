#ifndef REALSENSE_PARAMETER_HPP
#define REALSENSE_PARAMETER_HPP

#define NODE_NAME "realsense_camera"
#define DEPTH_IMAGE_TOPIC_NAME "depth_image"
#define COLOR_IMAGE_TOPIC_NAME "color_image"
#define PUBLISH_DEPTH false
#define PUBLISH_COLOR false

#define CABLE_LENGTH_TOPIC_NAME "cable_length"
#define SWAY_ANGLE_TOPIC_NAME "sway_angle"

#define MINIMUM_DISTANCE 0.2
#define MAXIMUM_DISTANCE 1.0

/* Depth frame resolution (width and height)
 * Option:
 * 1024x768
 * 640x480
 * 320x240*/
#define DEPTH_FRAME_WIDTH 320
#define DEPTH_FRAME_HEIGHT 240

/* Color frame resolution (width and height)
 * Option:
 * 1920x1080
 * 1280x720
 * 960x540*/
#define COLOR_FRAME_WIDTH 960
#define COLOR_FRAME_HEIGHT 540

// Callibration parameters
// Parameters for point projection
// Normal plane parameters from equation: Ax + By + Cz + D = 0
#define NORMAL_PLANE_A 0.0020000
#define NORMAL_PLANE_B -0.1010000
#define NORMAL_PLANE_C 0.0080000
#define NORMAL_PLANE_D -0.0099910
// Normal line parameters from equation: (x, y, z) = (x_1, y_1, z_1) + t * (A, B, C)
#define NORMAL_LINE_A 0.0000000
#define NORMAL_LINE_B -0.0789606
#define NORMAL_LINE_C -0.9968777
// Trolley origin (x0, y0, z0)
#define TROLLEY_ORIGIN_X 0.0180000
#define TROLLEY_ORIGIN_Y -0.0979498
#define TROLLEY_ORIGIN_Z 0.0077584

#define SWAY_ANGLE_OFFSET 0.0

#define CONTAINER_X_POSITION_MOVING_AVERAGE_WINDOW_SIZE 5
#define CONTAINER_Y_POSITION_MOVING_AVERAGE_WINDOW_SIZE 5
#define CONTAINER_Z_POSITION_MOVING_AVERAGE_WINDOW_SIZE 5

#endif // REALSENSE_PARAMETER_HPP