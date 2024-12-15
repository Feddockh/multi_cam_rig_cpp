#include "multi_cam_rig_cpp/zed_capture_node.hpp"

ZedCaptureNode::ZedCaptureNode()
    : Node("zed_capture_node"), camera_initialized_(false)
{
    // Declare and get parameters
    declare_parameter("save_images", false);
    declare_parameter("save_dir", "/tmp/multi_cam_images");
    declare_parameter("director_topic", "/multi_cam_rig/director");
    declare_parameter("left_image_topic", "/multi_cam_rig/zed/left_image");
    declare_parameter("right_image_topic", "/multi_cam_rig/zed/right_image");
    declare_parameter("imu_topic", "/multi_cam_rig/zed/imu");

    save_images_ = get_parameter("save_images").as_bool();
    save_dir_ = get_parameter("save_dir").as_string();
    std::string director_topic = get_parameter("director_topic").as_string();
    std::string left_image_topic = get_parameter("left_image_topic").as_string();
    std::string right_image_topic = get_parameter("right_image_topic").as_string();
    std::string imu_topic = get_parameter("imu_topic").as_string();

    director_publisher_ = create_publisher<std_msgs::msg::String>(director_topic, 10);
    director_subscriber_ = create_subscription<std_msgs::msg::String>(
        director_topic, 10, std::bind(&ZedCaptureNode::director_callback, this, std::placeholders::_1));
    left_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(left_image_topic, 10);
    right_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(right_image_topic, 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

    // Initialize the camera
    camera_initialized_ = initialize_camera();
    if (!camera_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZED camera.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "ZED camera initialized successfully.");
    }
}

ZedCaptureNode::~ZedCaptureNode()
{
    zed_.close();
}

bool ZedCaptureNode::initialize_camera()
{
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

    sl::ERROR_CODE err = zed_.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera: %s", sl::toString(err).c_str());
        return false;
    }

    zed_.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, -1);
    zed_.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);

    return true;
}

void ZedCaptureNode::director_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Check if the message starts with "capture "
    if (msg->data.rfind("capture ", 0) == 0)
    {

        // Extract the image ID from the message
        RCLCPP_INFO(this->get_logger(), "Received capture command: %s", msg->data.c_str());
        image_id_ = std::stoi(msg->data.substr(8));

        // Capture the image
        if (camera_initialized_)
        {
            capture_image();
            capture_imu_data();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ZED camera not initialized, cannot capture image.");
        }
    }
}

void ZedCaptureNode::capture_image()
{
    sl::RuntimeParameters runtime_params;
    if (zed_.grab(runtime_params) == sl::ERROR_CODE::SUCCESS)
    {
        sl::Mat zed_image;
        zed_.retrieveImage(zed_image, sl::VIEW::SIDE_BY_SIDE);

        // Convert to OpenCV format
        cv::Mat cvImage(
            zed_image.getHeight(),
            zed_image.getWidth(),
            CV_8UC4,
            zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));

        // Convert BGRA to BGR
        cv::Mat cvImageBGR;
        cv::cvtColor(cvImage, cvImageBGR, cv::COLOR_BGRA2BGR);

        // Split the image into left and right halves
        int width = cvImageBGR.cols / 2;
        left_image_ = cvImageBGR(cv::Rect(0, 0, width, cvImageBGR.rows));
        right_image_ = cvImageBGR(cv::Rect(width, 0, width, cvImageBGR.rows));

        // Convert to ROS message
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_image_).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_image_).toImageMsg();

        // Publish the images
        left_image_publisher_->publish(*left_msg);
        right_image_publisher_->publish(*right_msg);

        auto completion_msg = std_msgs::msg::String();
        completion_msg.data = "ZED Complete";
        director_publisher_->publish(completion_msg);
        RCLCPP_INFO(this->get_logger(), "Published ZED images.");

        // Save the images
        if (save_images_)
        {
            save_images();
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to grab image from ZED.");
    }
}

void ZedCaptureNode::capture_imu_data()
{
    if (zed_.getSensorsData(sensors_data_, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS)
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "zed_imu";

        // Fill orientation data
        imu_msg.orientation.x = sensors_data_.imu.pose.getOrientation().x;
        imu_msg.orientation.y = sensors_data_.imu.pose.getOrientation().y;
        imu_msg.orientation.z = sensors_data_.imu.pose.getOrientation().z;
        imu_msg.orientation.w = sensors_data_.imu.pose.getOrientation().w;

        // Fill angular velocity data
        imu_msg.angular_velocity.x = sensors_data_.imu.angular_velocity.x;
        imu_msg.angular_velocity.y = sensors_data_.imu.angular_velocity.y;
        imu_msg.angular_velocity.z = sensors_data_.imu.angular_velocity.z;

        // Fill linear acceleration data
        imu_msg.linear_acceleration.x = sensors_data_.imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = sensors_data_.imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = sensors_data_.imu.linear_acceleration.z;

        // Publish the IMU data
        imu_publisher_->publish(imu_msg);
        RCLCPP_INFO(this->get_logger(), "Published IMU data.");

        // Save IMU data if required
        if (save_images_)
        {
            save_imu_data();
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to retrieve IMU data from ZED.");
    }
}

void ZedCaptureNode::save_images()
{
    if (!std::filesystem::exists(save_dir_))
    {
        std::string left_image_path = save_dir_ + "/zed_left_" + std::to_string(image_id_) + ".jpg";
        std::string right_image_path = save_dir_ + "/zed_right_" + std::to_string(image_id_) + ".jpg";
        cv::imwrite(left_image_path, left_image_);
        cv::imwrite(right_image_path, right_image_);
        RCLCPP_INFO(this->get_logger(), "Saved ZED images.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Save directory does not exist: %s", save_dir_.c_str());
    }
}

void ZedCaptureNode::save_imu_data()
{
    if (!std::filesystem::exists(save_dir_))
    {
        std::filesystem::create_directories(save_dir_);
    }

    std::string imu_data_path = save_dir_ + "/imu_data_" + std::to_string(image_id_) + ".txt";
    std::ofstream imu_file(imu_data_path);

    if (imu_file.is_open())
    {
        imu_file << "Orientation: " << sensors_data_.imu.pose.getOrientation() << std::endl;
        imu_file << "Angular Velocity: " << sensors_data_.imu.angular_velocity << std::endl;
        imu_file << "Linear Acceleration: " << sensors_data_.imu.linear_acceleration << std::endl;
        imu_file.close();
        RCLCPP_INFO(this->get_logger(), "Saved IMU data.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to save IMU data to file.");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedCaptureNode>());
    rclcpp::shutdown();
    return 0;
}