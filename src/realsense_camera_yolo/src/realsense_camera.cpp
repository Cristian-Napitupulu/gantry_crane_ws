#include "realsense_camera/realsense_camera.hpp"

RealSenseCamera::RealSenseCamera() : Node("realsense_camera_node"),
                                     YOLO_client(nullptr),
                                     depthImagePublisher(nullptr),
                                     colorImagePublisher(nullptr),
                                     cableLengthPublisher(nullptr),
                                     swayAnglePublisher(nullptr),
                                     colorWidth(COLOR_FRAME_WIDTH),
                                     colorHeight(COLOR_FRAME_HEIGHT),
                                     depthWidth(DEPTH_FRAME_WIDTH),
                                     depthHeight(DEPTH_FRAME_HEIGHT),
                                     A(NORMAL_PLANE_A),
                                     B(NORMAL_PLANE_B),
                                     C(NORMAL_PLANE_C),
                                     D(NORMAL_PLANE_D),
                                     a(NORMAL_LINE_A),
                                     b(NORMAL_LINE_B),
                                     c(NORMAL_LINE_C),
                                     x0(TROLLEY_ORIGIN_X),
                                     y0(TROLLEY_ORIGIN_Y),
                                     z0(TROLLEY_ORIGIN_Z),
                                     projector(A, B, C, D, a, b, c, x0, y0, z0)
{
    initializeParameters();
    printParameters();
    initializePublishers();
    initializeRealSenseCamera();

    bool isYOLOServerIsUp = initializeYOLOClient();

    rclcpp::Time time_last = rclcpp::Clock().now();
    float loop_rate_last = 0;
    while (rclcpp::ok())
    {
        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "Waiting for new frames...");

        // Wait for next set of frames
        rs2::frameset frames = pipeline.wait_for_frames();

        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "New frames have been captured.");

        // Send request for YOLO
        if (isYOLOServerIsUp)
        {
            sendRequestToYOLO(frames);
        }
        // Publish images if needed
        if (publishDepth || publishColor)
        {
            publishImage(frames);
        }
        else if ((!isYOLOServerIsUp && !publishDepth && !publishColor))
        {
            // Server is down and no need to publish images
            RCLCPP_ERROR(this->get_logger(), "YOLO server is down and no need to publish images.");
            RCLCPP_ERROR(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }

        // Check log level
        if (rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) == RCUTILS_LOG_SEVERITY_DEBUG)
        {
            // Print loop rate in miliseconds
            rclcpp::Time time_now = rclcpp::Clock().now();
            float loop_rate_now = static_cast<float>(1000000000 / (time_now - time_last).nanoseconds());
            RCLCPP_INFO(this->get_logger(), "Loop rate: \t %.2f \t per second", (loop_rate_now + loop_rate_last) / 2);
            time_last = time_now;
            loop_rate_last = loop_rate_now;
        }
    }
}

void RealSenseCamera::initializeParameters()
{
    // Declare parameters and the default value
    declare_parameter("depth_image_topic", DEPTH_IMAGE_TOPIC);
    declare_parameter("color_image_topic", COLOR_IMAGE_TOPIC);
    declare_parameter("minimum_distance", MINIMUM_DISTANCE); // meters
    declare_parameter("maximum_distance", MAXIMUM_DISTANCE); // meters
    declare_parameter("publish_depth", PUBLISH_DEPTH);
    declare_parameter("publish_color", PUBLISH_COLOR);

    // Get parameters
    depthImageTopic = get_parameter("depth_image_topic").as_string();
    colorImageTopic = get_parameter("color_image_topic").as_string();
    minimumDistance = get_parameter("minimum_distance").as_double();
    maximumDistance = get_parameter("maximum_distance").as_double();
    publishDepth = get_parameter("publish_depth").as_bool();
    publishColor = get_parameter("publish_color").as_bool();

    // Check log level
    if (rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) == RCUTILS_LOG_SEVERITY_DEBUG)
    {
        // Enable publish depth and color
        publishDepth = true;
        publishColor = true;
    }
}

bool RealSenseCamera::initializeYOLOClient()
{
    // Initialize YOLO client
    YOLO_client = this->create_client<RealsenseYOLO>("realsense_yolo");

    // Wait for YOLO server
    if (YOLO_client->wait_for_service(std::chrono::seconds(15)))
    {
        RCLCPP_INFO(get_logger(), "YOLO service server is up.");
        return true;
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "YOLO service server is not available after waiting.");
        return false;
    }
}

void RealSenseCamera::initializePublishers()
{
    cableLengthPublisher = create_publisher<std_msgs::msg::Float64>("cable_length", 10);
    swayAnglePublisher = create_publisher<std_msgs::msg::Float64>("sway_angle", 10);

    if (publishDepth)
    {
        depthImagePublisher = create_publisher<sensor_msgs::msg::Image>(depthImageTopic, 10);
    }
    if (publishColor)
    {
        colorImagePublisher = create_publisher<sensor_msgs::msg::Image>(colorImageTopic, 10);
    }

    RCLCPP_DEBUG(get_logger(), "RealSense Camera Node has been initialized.");
}

void RealSenseCamera::initializeRealSenseCamera()
{
    rs2::config configuration;
    configuration.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30);
    configuration.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_BGR8, 30);
    pipeline.start(configuration);

    rs2::depth_sensor depthSensor = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
    depthSensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0);
    depthSensor.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimumDistance));

    RCLCPP_DEBUG(get_logger(), "RealSense Camera has been configured.");
}

rs2::depth_frame RealSenseCamera::processDepthFrame(rs2::depth_frame depth_frame)
{
    rs2::threshold_filter threshold_filter;
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimumDistance));
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, static_cast<float>(maximumDistance));
    return threshold_filter.process(depth_frame);
}

void RealSenseCamera::sendRequestToYOLO(rs2::frameset frames)
{
    // Get depth frame
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    depth_frame = processDepthFrame(depth_frame);

    // Convert depth frame to opencv matrix
    cv::Mat depth_image(cv::Size(depthWidth, depthHeight), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    auto depth_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();

    // Send request to YOLO
    auto request = std::make_shared<RealsenseYOLO::Request>();
    request->depth_image = *depth_image_message.get();
    auto result = YOLO_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Get response
        auto response = result.get();

        // Get bounding box pixel
        containerBoundingBoxPixel[0] = response->x1;
        containerBoundingBoxPixel[1] = response->y1;
        containerBoundingBoxPixel[2] = response->x2;
        containerBoundingBoxPixel[3] = response->y2;

        // Print bounding box pixel for debugging
        RCLCPP_INFO(this->get_logger(), "Bounding box pixel: \t %d \t %d \t %d \t %d", containerBoundingBoxPixel[0], containerBoundingBoxPixel[1], containerBoundingBoxPixel[2], containerBoundingBoxPixel[3]);

        // Project bounding box pixel to point
        projectContainerPixelToPoint(depth_frame);

        // Publish cable length and sway angle
        publishCableLengthAndSwayAngle();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to call YOLO service.");
    }

    // Print something for debugging
    RCLCPP_INFO_ONCE(this->get_logger(), "YOLO request has been sent.");
}

void RealSenseCamera::projectContainerPixelToPoint(rs2::depth_frame depth_frame)
{
    // Get center pixel
    float center_pixel[2] = {(float)(containerBoundingBoxPixel[0] + containerBoundingBoxPixel[2]) / 2,
                             (float)(containerBoundingBoxPixel[1] + containerBoundingBoxPixel[3]) / 2};
    float depth_value = depth_frame.get_distance(center_pixel[0], center_pixel[1]);
    // Get depth value
    rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();

    rs2_deproject_pixel_to_point(containerCenterPoint, &depth_intrinsics, center_pixel, depth_value);

    // Print container center point for debugging
    RCLCPP_INFO(get_logger(), "Container center point: ( %.5f , %.5f , %.5f )",
                containerCenterPoint[0], containerCenterPoint[1], containerCenterPoint[2]);
}

void RealSenseCamera::publishCableLengthAndSwayAngle()
{
    // Create x, y, z variables
    double x = containerCenterPoint[0];
    double y = containerCenterPoint[1];
    double z = containerCenterPoint[2];

    // Project point to normal plane
    projector.projectPoint(x, y, z);
    // Get the distance from trolley origin to the projected point
    double cable_length = projector.getDistance(x, y, z);
    // Get the sway angle from the normal line
    double sway_angle = projector.getAngle(x, y, z);
    // Convert to degree
    sway_angle = sway_angle * 180 / M_PI;

    // Publish cable length
    auto cable_length_message = std_msgs::msg::Float64();
    cable_length_message.data = cable_length;
    cableLengthPublisher->publish(cable_length_message);

    // Publish sway angle
    auto sway_angle_message = std_msgs::msg::Float64();
    sway_angle_message.data = sway_angle;
    swayAnglePublisher->publish(sway_angle_message);

    // Print cable length and sway angle for debugging
    RCLCPP_INFO(this->get_logger(), "Cable length: \t %.5f \t meters, Sway angle: \t %.5f \t degrees", cable_length, sway_angle);
}

void RealSenseCamera::publishImage(rs2::frameset frames)
{
    // Process depth image
    if (publishDepth)
    {
        // Get depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        depth_frame = processDepthFrame(depth_frame);

        // Convert depth frame to opencv matrix
        cv::Mat depth_image(cv::Size(depthWidth, depthHeight), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        auto depth_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
        depth_image_message->header.stamp = rclcpp::Clock().now();

        // Publish depth image
        depthImagePublisher->publish(*depth_image_message.get());

        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "Depth image has been published.");
    }

    // Process color image
    if (publishColor)
    {
        // Get color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Convert color frame to opencv matrix
        cv::Mat color_image(cv::Size(colorWidth, colorHeight), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        auto color_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        color_image_message->header.stamp = rclcpp::Clock().now();

        // Publish color image
        colorImagePublisher->publish(*color_image_message.get());

        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "Color image has been published.");
    }
}

void RealSenseCamera::printParameters()
{
    // Print parameters
    RCLCPP_INFO(this->get_logger(), "depth_image_topic: %s", depthImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "color_image_topic: %s", colorImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "minimum_distance: %.3f meters", minimumDistance);
    RCLCPP_INFO(this->get_logger(), "maximum_distance: %.3f meters", maximumDistance);
    RCLCPP_INFO(this->get_logger(), "publish_depth: %s", publishDepth ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "publish_color: %s", publishColor ? "true" : "false");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseCamera>());
    rclcpp::shutdown();
    return 0;
}