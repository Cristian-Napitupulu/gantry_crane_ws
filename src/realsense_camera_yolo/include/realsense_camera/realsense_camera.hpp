#ifndef REAL_SENSE_CAMERA_HPP
#define REAL_SENSE_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "realsense_camera_yolo_interfaces/srv/realsense_yolo.hpp"

#include "point_projections/point_projections.hpp"

class RealSenseCamera : public rclcpp::Node
{
public:
    using RealsenseYOLO = realsense_camera_yolo_interfaces::srv::RealsenseYOLO;

    RealSenseCamera();

private:
    // Declare action client
    rclcpp::Client<RealsenseYOLO>::SharedPtr YOLO_client;

    // Declare publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cable_length_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sway_angle_publisher;

    // Declare parameters
    std::string depth_image_topic;
    std::string color_image_topic;
    double minimum_distance;
    double maximum_distance;
    bool publish_depth;
    bool publish_color;

    float container_center_point[3];     // x, y, z
    int container_bounding_box_pixel[4]; // x_1, y_1, x_2, y_2

    void project_container_pixel_to_point(rs2::depth_frame depth_frame);

    void publish_cable_length_and_sway_angle();

    // Declare realsense camera
    rs2::pipeline pipeline;
    rs2::config configuration;
    rs2_intrinsics depth_intrinsics;
    rs2::threshold_filter threshold_filter;
    rs2::frame stored_depth_frame;

    bool depth_frame_aquired = false;

    // Color frame resolution (width and height)
    const int color_width = 960;
    const int color_height = 540;
    /*Option:
     *1920x1080
     *1280x720
     *960x540*/

    // Depth frame resolution (width and height)
    const int depth_width = 320;
    const int depth_height = 240;
    /*Option:
     *1024x768
     *640x480
     *320x240*/

    // Declare functions
    void send_request_to_YOLO(rs2::frameset frames);

    void publish_image(rs2::frameset frames);

    // Transformation parameter to transform the point cloud from the camera frame to the trolley frame
    double dx = 0.02;
    double dy = -0.10;
    double dz = 0.0;
    double theta_x = -6.8;
    double theta_y = -4.0;
    double theta_z = 0.0;

    // Parameters for the point projection
    // Plane parameters from equation: Ax + By + Cz + D = 0
    // 3D Line parameters from equation: <x, y, z> = <x_1, y_1, z_1> + t<a, b, c>
    // New origin of the coordinate system (x0, y0, z0)
    double A = 0.0;
    double B = 0.101;
    double C = -0.01100000000000001;
    double D = 0.009854000000000005;

    double a = 0.06612969055395865;
    double b = 0.10803365288517996;
    double c = 0.9919453583093789;

    double x0 = -0.006670616393108123;
    double y0 = -0.09637278915705783;
    double z0 = 0.01094075410337858;
    pointProjections projector = pointProjections(A, B, C, D, a, b, c, x0, y0, z0);
};

#endif // REAL_SENSE_CAMERA_HPP