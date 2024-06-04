#include "realsense_camera/realsense_camera.hpp"
#include "realsense_parameter.hpp"

#define NO_RESULT 0
#define YOLO_RESULT 1
#define CORNER_RESULT 2

RealSenseCamera::RealSenseCamera() : Node(NODE_NAME),
                                     YOLO_client(nullptr),
                                     depthImagePublisher(nullptr),
                                     colorImagePublisher(nullptr),
                                     cableLengthPublisher(nullptr),
                                     swayAnglePublisher(nullptr),
                                     colorWidth(COLOR_FRAME_WIDTH),
                                     colorHeight(COLOR_FRAME_HEIGHT),
                                     depthWidth(DEPTH_FRAME_WIDTH),
                                     depthHeight(DEPTH_FRAME_HEIGHT),
                                     projector(NORMAL_PLANE_A, NORMAL_PLANE_B, NORMAL_PLANE_C, NORMAL_PLANE_D,
                                               NORMAL_LINE_A, NORMAL_LINE_B, NORMAL_LINE_C,
                                               TROLLEY_ORIGIN_X, TROLLEY_ORIGIN_Y, TROLLEY_ORIGIN_Z),
                                     cableLengthMovingAverage(CABLE_LENGTH_MOVING_AVERAGE_WINDOW_SIZE),
                                     swayAngleMovingAverage(SWAY_ANGLE_MOVING_AVERAGE_WINDOW_SIZE),
                                     DC_OffsetSwayAngleMovingAverage(DC_OFFSET_SWAY_ANGLE_MOVING_AVERAGE_WINDOW_SIZE),
                                     executionTime(30)
{

    initializeRealSenseCamera();

    initializeParameters();
    printParameters();
    initializePublishers();

    bool isYOLOServerIsUp = initializeYOLOClient();

    while (rclcpp::ok())
    {
        // Print loop rate in miliseconds
        rclcpp::Time start_time = rclcpp::Clock().now();

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

        rclcpp::Time end_time = rclcpp::Clock().now();
        int execution_time_ = static_cast<int>((end_time - start_time).seconds() * 1000);
        executionTime.addValue(execution_time_);
    }
}

void RealSenseCamera::initializeParameters()
{
    // Declare parameters and the default value
    declare_parameter("depth_image_topic", DEPTH_IMAGE_TOPIC_NAME);
    declare_parameter("color_image_topic", COLOR_IMAGE_TOPIC_NAME);
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
    cableLengthPublisher = create_publisher<std_msgs::msg::Float32>(CABLE_LENGTH_TOPIC_NAME, 10);
    swayAnglePublisher = create_publisher<std_msgs::msg::Float32>(SWAY_ANGLE_TOPIC_NAME, 10);
    RCLCPP_INFO(get_logger(), "Cable length and sway angle publishers have been initialized.");

    if (publishDepth)
    {
        depthImagePublisher = create_publisher<sensor_msgs::msg::Image>(depthImageTopic, 10);
        RCLCPP_INFO(get_logger(), "Depth image publisher has been initialized.");
    }
    if (publishColor)
    {
        colorImagePublisher = create_publisher<sensor_msgs::msg::Image>(colorImageTopic, 10);
        RCLCPP_INFO(get_logger(), "Color image publisher has been initialized.");
    }
}

void RealSenseCamera::initializeRealSenseCamera()
{
    rs2::config configuration;
    configuration.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30);
    if (publishColor)
    {
        configuration.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_BGR8, 30);
    }
    pipeline.start(configuration);

    rs2::depth_sensor depthSensor = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
    depthSensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0);
    depthSensor.set_option(RS2_OPTION_MIN_DISTANCE, static_cast<float>(minimumDistance));

    RCLCPP_INFO(get_logger(), "RealSense camera has been initialized.");
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

    // Print something for debugging
    RCLCPP_INFO_ONCE(this->get_logger(), "YOLO request has been sent.");

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Get response
        auto response = result.get();

        if (response->id == NO_RESULT)
        {
            RCLCPP_WARN(this->get_logger(), "No container detected.");
            return;
        }
        else if (response->id == YOLO_RESULT)
        {
            // Get bounding box pixel
            containerBoundingBoxPixel[0] = response->x1;
            containerBoundingBoxPixel[1] = response->y1;
            containerBoundingBoxPixel[2] = response->x2;
            containerBoundingBoxPixel[3] = response->y2;
        }
        else if (response->id == CORNER_RESULT)
        {
            // Get bounding box pixel
            containerBoundingBoxPixel[0] = response->x1;
            containerBoundingBoxPixel[1] = response->y1;
            containerBoundingBoxPixel[2] = response->x2;
            containerBoundingBoxPixel[3] = response->y2;
        }

        // Project bounding box pixel to point
        projectContainerPixelToPoint(depth_frame);

        // Publish cable length and sway angle
        publishCableLengthAndSwayAngle();
    }

    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to call YOLO service.");
    }
}

std::vector<rs2::vertex> RealSenseCamera::generatePointCloudInsideBoundingBox(rs2::depth_frame depth_frame)
{
    // Get depth value
    rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();

    // Get bounding box pixel
    int x1 = containerBoundingBoxPixel[0];
    int y1 = containerBoundingBoxPixel[1];
    int x2 = containerBoundingBoxPixel[2];
    int y2 = containerBoundingBoxPixel[3];

    std::vector<rs2::vertex> pointCloud;

    // Get depth frame dimensions
    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    // Iterate over pixels inside the bounding box
    for (int y = y1; y < y2; ++y)
    {
        for (int x = x1; x < x2; ++x)
        {
            // Ensure the pixel coordinates are within the frame dimensions
            if (x >= 0 && x < width && y >= 0 && y < height)
            {
                // Get the depth value at the current pixel
                float depthValue = depth_frame.get_distance(x, y);

                // Check if the depth value is valid
                if (depthValue > minimumDistance && depthValue < maximumDistance)
                {
                    float coordinates[2] = {(float)x, (float)y};

                    float depthPoint[3];
                    rs2_deproject_pixel_to_point(depthPoint, &depth_intrinsics, coordinates, depthValue);
                    // Map pixel coordinates to 3D point in camera coordinates
                    rs2::vertex point = {static_cast<float>(depthPoint[0]),
                                         static_cast<float>(depthPoint[1]),
                                         static_cast<float>(depthPoint[2])};
                    pointCloud.push_back(point);
                }
            }
        }
    }

    return pointCloud;
}

cv::Mat RealSenseCamera::pointCloudToMat(std::vector<rs2::vertex> pointCloud)
{
    int size = static_cast<int>(pointCloud.size());
    // Create a matrix with the same number of rows as the point cloud
    cv::Mat pointCloudMat(size, 3, CV_32FC1);

    // Iterate over the point cloud
    for (int i = 0; i < size; ++i)
    {
        // Get the coordinates of the current point
        float x = pointCloud[i].x;
        float y = pointCloud[i].y;
        float z = pointCloud[i].z;

        // Store the coordinates in the matrix
        pointCloudMat.at<float>(i, 0) = x;
        pointCloudMat.at<float>(i, 1) = y;
        pointCloudMat.at<float>(i, 2) = z;
    }

    return pointCloudMat;
}

void RealSenseCamera::projectContainerPixelToPoint(rs2::depth_frame depth_frame)
{
    // Get center pixel
    float center_pixel[2] = {(float)(containerBoundingBoxPixel[0] + containerBoundingBoxPixel[2]) / 2,
                             (float)(containerBoundingBoxPixel[1] + containerBoundingBoxPixel[3]) / 2};
    if (center_pixel[0] < 0 || center_pixel[0] > depthWidth || center_pixel[1] < 0 || center_pixel[1] > depthHeight)
    {
        RCLCPP_INFO(this->get_logger(), "Center pixel is out of range.");
        return;
    }

    float depth_value = depth_frame.get_distance(center_pixel[0], center_pixel[1]);
    // Get depth value
    rs2::video_stream_profile depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_profile.get_intrinsics();

    float depthPoint[3];
    rs2_deproject_pixel_to_point(depthPoint, &depth_intrinsics, center_pixel, depth_value);

    // containerXPosition.addValue(depthPoint[0]);
    // containerYPosition.addValue(depthPoint[1]);
    // containerZPosition.addValue(depthPoint[2]);

    // containerCenterPoint[0] = containerXPosition.getMovingAverage();
    // containerCenterPoint[1] = containerYPosition.getMovingAverage();
    // containerCenterPoint[2] = containerZPosition.getMovingAverage();

    containerCenterPoint[0] = depthPoint[0];
    containerCenterPoint[1] = depthPoint[1];
    containerCenterPoint[2] = depthPoint[2];

    // Print container center point for debugging
    RCLCPP_INFO(get_logger(), "Container center point (x, y, z) (meters): (%.5f, %.5f, %.5f)",
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
    cableLengthMovingAverage.addValue(cable_length);

    // Get the sway angle from the normal line
    double sway_angle = projector.getAngle(x, y, z);
    // Convert to degree
    sway_angle = sway_angle * 180 / M_PI + SWAY_ANGLE_OFFSET;

    swayAngleMovingAverage.addValue(sway_angle);

    double sway_angle_ = swayAngleMovingAverage.getMovingAverage();

    DC_OffsetSwayAngleMovingAverage.addValue(sway_angle_);

    sway_angle_ = sway_angle_ - DC_OffsetSwayAngleMovingAverage.getMovingAverage();

    // Publish cable length
    auto cable_length_message = std_msgs::msg::Float32();
    // cable_length_message.data = cable_length;
    cable_length_message.data = cableLengthMovingAverage.getMovingAverage();
    cableLengthPublisher->publish(cable_length_message);

    // Publish sway angle
    auto sway_angle_message = std_msgs::msg::Float32();
    sway_angle_message.data = sway_angle_;
    swayAnglePublisher->publish(sway_angle_message);

    int execution_time = executionTime.getMovingAverage();
    // Print cable length and sway angle for debugging
    RCLCPP_INFO(this->get_logger(), "Publishing cable length: %.5f meters, and sway angle: %.5f degrees. Execution time: %d ms.", cable_length, sway_angle, execution_time);
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
    try
    {
        rclcpp::spin(std::make_shared<RealSenseCamera>());
    }
    catch (const rs2::error &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RealSense error calling %s: %s", e.get_failed_function().c_str(), e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}