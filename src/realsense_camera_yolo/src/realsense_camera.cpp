#include "realsense_camera/realsense_camera.hpp"

RealSenseCamera::RealSenseCamera() : Node("realsense_camera_node")
{
    // Declare parameters and the default value
    this->declare_parameter("depth_image_topic", "depth_image");
    this->declare_parameter("color_image_topic", "color_image");
    this->declare_parameter("minimum_distance", 0.15); // meters
    this->declare_parameter("maximum_distance", 1.0); // meters
    this->declare_parameter("publish_depth", false);
    this->declare_parameter("publish_color", false);

    // Get parameters
    depth_image_topic = this->get_parameter("depth_image_topic").as_string();
    color_image_topic = this->get_parameter("color_image_topic").as_string();
    minimum_distance = this->get_parameter("minimum_distance").as_double();
    maximum_distance = this->get_parameter("maximum_distance").as_double();
    publish_depth = this->get_parameter("publish_depth").as_bool();
    publish_color = this->get_parameter("publish_color").as_bool();

    // Check log level
    if (rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) == RCUTILS_LOG_SEVERITY_DEBUG)
    {
        // Enable publish depth and color
        publish_depth = true;
        publish_color = true;
    }

    // Print parameters
    RCLCPP_INFO(this->get_logger(), "depth_image_topic: %s", depth_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "color_image_topic: %s", color_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "minimum_distance: %f", minimum_distance);
    RCLCPP_INFO(this->get_logger(), "maximum_distance: %f", maximum_distance);
    RCLCPP_INFO(this->get_logger(), "publish_depth: %s", publish_depth ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "publish_color: %s", publish_color ? "true" : "false");

    // Initialize YOLO client
    YOLO_client = this->create_client<RealsenseYOLO>("realsense_yolo");

    // Wait for YOLO service server
    bool is_YOLO_server_up = false;
    if (this->YOLO_client->wait_for_service(std::chrono::seconds(15)))
    {
        RCLCPP_INFO(this->get_logger(), "YOLO service server is up.");
        is_YOLO_server_up = true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "YOLO service server is not available after waiting.");
    }

    // Initialize publishers
    cable_length_publisher = create_publisher<std_msgs::msg::Float32>("cable_length", 10);
    sway_angle_publisher = create_publisher<std_msgs::msg::Float32>("sway_angle", 10);

    if (publish_depth)
    {
        depth_image_publisher = create_publisher<sensor_msgs::msg::Image>(depth_image_topic, 10);
    }
    if (publish_color)
    {
        color_image_publisher = create_publisher<sensor_msgs::msg::Image>(color_image_topic, 10);
    }

    // Print something for debugging
    RCLCPP_DEBUG(this->get_logger(), "RealSense Camera Node has been initialized.");

    // Initialize realsense camera
    configuration.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, 30);
    configuration.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, 30);
    pipeline.start(configuration);

    // Print something for debugging
    RCLCPP_DEBUG(this->get_logger(), "RealSense Camera has been initialized.");

    // Disable Max Usable Range
    rs2::depth_sensor depth_sensor = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0);
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimum_distance));

    // Print something for debugging
    RCLCPP_DEBUG(this->get_logger(), "RealSense Camera has been configured.");

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
        if (is_YOLO_server_up){
            send_request_to_YOLO(frames);
        }

        // Publish images if needed
        if (publish_depth || publish_color)
        {
            publish_image(frames);
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

void RealSenseCamera::send_request_to_YOLO(rs2::frameset frames)
{
    // Get depth frame
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    // Apply filters to depth frame
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimum_distance));
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, static_cast<float>(maximum_distance));
    depth_frame = threshold_filter.process(depth_frame);

    if (!depth_frame_aquired)
    {
        stored_depth_frame = depth_frame;
        depth_frame_aquired = true;
    }
    // Convert depth frame to opencv matrix
    cv::Mat depth_image(cv::Size(depth_width, depth_height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

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
        container_bounding_box_pixel[0] = response->x1;
        container_bounding_box_pixel[1] = response->y1;
        container_bounding_box_pixel[2] = response->x2;
        container_bounding_box_pixel[3] = response->y2;

        // Print bounding box pixel for debugging
        RCLCPP_INFO(this->get_logger(), "Bounding box pixel: \t %d \t %d \t %d \t %d", container_bounding_box_pixel[0], container_bounding_box_pixel[1], container_bounding_box_pixel[2], container_bounding_box_pixel[3]);

        // Project bounding box pixel to point
        project_container_pixel_to_point(depth_frame);

        // Publish cable length and sway angle
        publish_cable_length_and_sway_angle();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to call YOLO service.");
    }

    // Print something for debugging
    RCLCPP_INFO_ONCE(this->get_logger(), "YOLO request has been sent.");
}

void RealSenseCamera::project_container_pixel_to_point(rs2::depth_frame depth_frame)
{
    // Get center pixel
    float center_pixel[2] = {(float)(container_bounding_box_pixel[0] + container_bounding_box_pixel[2]) / 2, (float)(container_bounding_box_pixel[1] + container_bounding_box_pixel[3]) / 2};
    float depth_value = depth_frame.get_distance(center_pixel[0], center_pixel[1]);
    // Get depth value
    rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();

    rs2_deproject_pixel_to_point(container_center_point, &depth_intrinsics, center_pixel, depth_value);

    // Print container center point for debugging
    RCLCPP_INFO(this->get_logger(), "Container center point: \t %.2f \t %.2f \t %.2f", container_center_point[0], container_center_point[1], container_center_point[2]);
}

void RealSenseCamera::publish_cable_length_and_sway_angle()
{
    float x = container_center_point[0];
    float y = container_center_point[1];
    float z = container_center_point[2];

    float cable_length = projector.projectPointAndGetDistance(x, y, z);
    float sway_angle = projector.projectPointAndGetAngle(x, y, z);
    // Convert to degree
    sway_angle = sway_angle * 180 / M_PI;

    auto cable_length_message = std_msgs::msg::Float32();
    cable_length_message.data = cable_length;
    cable_length_publisher->publish(cable_length_message);

    auto sway_angle_message = std_msgs::msg::Float32();
    sway_angle_message.data = sway_angle;
    sway_angle_publisher->publish(sway_angle_message);

    // Print cable length and sway angle for debugging
    RCLCPP_INFO(this->get_logger(), "Cable length: \t %.2f \t meters, Sway angle: \t %.2f \t degrees", cable_length, sway_angle);
}

void RealSenseCamera::publish_image(rs2::frameset frames)
{
    // Process depth image
    if (publish_depth)
    {
        // Get depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // Apply filters to depth frame
        threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimum_distance));
        threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, static_cast<float>(maximum_distance));
        depth_frame = threshold_filter.process(depth_frame);

        // Convert depth frame to opencv matrix
        cv::Mat depth_image(cv::Size(depth_width, depth_height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        auto depth_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
        depth_image_message->header.stamp = rclcpp::Clock().now();

        // Publish depth image
        depth_image_publisher->publish(*depth_image_message.get());

        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "Depth image has been published.");
    }

    // Process color image
    if (publish_color)
    {
        // Get color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Convert color frame to opencv matrix
        cv::Mat color_image(cv::Size(color_width, color_height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        auto color_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        color_image_message->header.stamp = rclcpp::Clock().now();

        // Publish color image
        color_image_publisher->publish(*color_image_message.get());

        // Print something for debugging
        RCLCPP_DEBUG(this->get_logger(), "Color image has been published.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseCamera>());
    rclcpp::shutdown();
    return 0;
}