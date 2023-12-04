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
#include "moving_average/moving_average.hpp"

class RealSenseCamera : public rclcpp::Node
{
public:
    using RealsenseYOLO = realsense_camera_yolo_interfaces::srv::RealsenseYOLO;

    RealSenseCamera();

private:
    void initializeParameters();
    void printParameters();
    void initializePublishers();
    void initializeRealSenseCamera();
    bool initializeYOLOClient();

    void sendRequestToYOLO(rs2::frameset frames);
    void projectContainerPixelToPoint(rs2::depth_frame depth_frame);
    void publishCableLengthAndSwayAngle();
    void publishImage(rs2::frameset frames);
    rs2::depth_frame processDepthFrame(rs2::depth_frame depth_frame);

    // Declare action client
    rclcpp::Client<RealsenseYOLO>::SharedPtr YOLO_client;

    // Declare publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorImagePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cableLengthPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr swayAnglePublisher;

    // Declare parameters
    std::string depthImageTopic;
    std::string colorImageTopic;
    double minimumDistance;
    double maximumDistance;
    bool publishDepth;
    bool publishColor;

    float containerCenterPoint[3];  // x, y, z
    int containerBoundingBoxPixel[4];   // x1, y1, x2, y2

    // Declare realsense camera
    rs2::pipeline pipeline;

    const int colorWidth, colorHeight;
    const int depthWidth, depthHeight; 

    pointProjections projector;
    MovingAverage containerXPosition;
    MovingAverage containerYPosition;
    MovingAverage containerZPosition;
};

#endif // REAL_SENSE_CAMERA_HPP