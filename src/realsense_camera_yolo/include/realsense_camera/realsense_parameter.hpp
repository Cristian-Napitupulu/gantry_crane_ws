#ifndef REALSENSE_PARAMETER_HPP
#define REALSENSE_PARAMETER_HPP

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

#define DEPTH_IMAGE_TOPIC "depth_image"
#define COLOR_IMAGE_TOPIC "color_image"
#define PUBLISH_DEPTH true
#define PUBLISH_COLOR true

// Callibration parameters
// Parameters for point projection
// Normal plane parameters from equation: Ax + By + Cz + D = 0
#define NORMAL_PLANE_A 0.0
#define NORMAL_PLANE_B 0.101
#define NORMAL_PLANE_C -0.01100000000000001
#define NORMAL_PLANE_D 0.009854000000000005
// Normal line parameters from equation: (x, y, z) = (x_1, y_1, z_1) + t * (A, B, C)
#define NORMAL_LINE_A 0.06612969055395865
#define NORMAL_LINE_B 0.10803365288517996
#define NORMAL_LINE_C 0.9919453583093789
// Trolley origin (x0, y0, z0)
#define TROLLEY_ORIGIN_X -0.006670616393108123
#define TROLLEY_ORIGIN_Y -0.09637278915705783
#define TROLLEY_ORIGIN_Z 0.01094075410337858

#endif // REALSENSE_PARAMETER_HPP