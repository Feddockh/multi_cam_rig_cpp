#include "multi_cam_rig_cpp/ximea_capture_node.hpp"

using namespace std::chrono_literals;

XimeaCaptureNode::XimeaCaptureNode()
    : Node("ximea_capture_node"), xi_handle_(nullptr), camera_initialized_(false)
{
    // Declare and get parameters
    declare_parameter<std::string>("director_topic", "/multi_cam_rig/director");
    declare_parameter<std::string>("ximea_image_topic", "/multi_cam_rig/ximea/image");

    std::string director_topic = get_parameter("director_topic").as_string();
    std::string image_topic = get_parameter("ximea_image_topic").as_string();

    // Initialize Publishers
    director_publisher_ = create_publisher<std_msgs::msg::String>(director_topic, 10);
    director_subscriber_ = create_subscription<std_msgs::msg::String>(
        director_topic, 10,
        std::bind(&XimeaCaptureNode::director_callback, this, std::placeholders::_1));
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

    RCLCPP_INFO(this->get_logger(), "XimeaCaptureNode initialized, subscribed to: %s", director_topic.c_str());

    // Initialize the camera
    camera_initialized_ = initialize_camera();
    if (!camera_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize XIMEA camera.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "XIMEA camera initialized successfully.");
    }
}

XimeaCaptureNode::~XimeaCaptureNode()
{
    if (camera_initialized_ && xi_handle_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping acquisition and closing XIMEA camera.");
        XI_RETURN stat = xiStopAcquisition(xi_handle_);
        if (stat != XI_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop acquisition.");
        }

        stat = xiCloseDevice(xi_handle_);
        if (stat != XI_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to close XIMEA camera.");
        }
    }
}

bool XimeaCaptureNode::initialize_camera()
{
    XI_RETURN stat = XI_OK;

    // Open the first camera
    RCLCPP_INFO(this->get_logger(), "Opening first XIMEA camera...");
    stat = xiOpenDevice(0, &xi_handle_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open XIMEA camera.");
        return false;
    }

    // Set auto exposure/gain
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG, XI_ON);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set auto exposure/gain.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Correct sensor defects
    stat = xiSetParamInt(xi_handle_, XI_PRM_SENS_DEFECTS_CORR, XI_ON);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to correct sensor defects.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Set the downsample factor
    stat = xiSetParamInt(xi_handle_, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set downsample factor.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Set image format
    xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set image format.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Start acquisition
    RCLCPP_INFO(this->get_logger(), "Starting acquisition...");
    stat = xiStartAcquisition(xi_handle_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start acquisition.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    return true;
}

void XimeaCaptureNode::director_callback(const std_msgs::msg::String::SharedPtr msg)
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
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "XIMEA camera not initialized, cannot capture image.");
        }
    }
}

void XimeaCaptureNode::capture_image()
{
    if (!camera_initialized_ || xi_handle_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Camera is not initialized.");
        return;
    }

    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    XI_RETURN stat = xiGetImage(xi_handle_, 1000, &image);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get image.");
        return;
    }

    if (image.bp == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Image buffer is null.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Image Width: %d, Height: %d, Format: %d", image.width, image.height, image.frm);

    // Ensure the image format matches XI_MONO8 (1 channel)
    cv::Mat cvImageMono(image.height, image.width, CV_8UC1, image.bp);

    // Clone the image to ensure data integrity
    image_ = cvImageMono.clone();

    // Create ROS Image message
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_).toImageMsg();

    // Publish the image
    image_publisher_->publish(*image_msg);

    auto completion_msg = std_msgs::msg::String();
    completion_msg.data = "XIMEA Complete";
    director_publisher_->publish(completion_msg);
    RCLCPP_INFO(this->get_logger(), "Published XIMEA image.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XimeaCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
