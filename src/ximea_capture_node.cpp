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

    // Set the downsample factor
    stat = xiSetParamInt(xi_handle_, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set downsample factor.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
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

    // Cannot get auto white balance to work
    // // Auto white balance
    // stat = xiSetParamInt(xi_handle_, XI_PRM_AUTO_WB, 1);
    // if (stat != XI_OK)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set auto white balance.");
    //     xiCloseDevice(xi_handle_);
    //     xi_handle_ = nullptr;
    //     return false;
    // }

    // Set image format
    xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set image format.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // // Get the FFC files
    // std::string dark_file, mid_file;
    // if (!get_ffc(dark_file, mid_file))
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to get FFC files. Check the shading directory.");
    //     xiCloseDevice(xi_handle_);
    //     xi_handle_ = nullptr;
    //     return false;
    // }

    /// For the life of me I could not get the FFC files to work. I tried everything I could think of.
    // // Set the ffc dark field frame
    // char dark_file[] = "dark.tif";
    // stat = xiSetParamString(xi_handle_, XI_PRM_FFC_DARK_FIELD_FILE_NAME, dark_file, strlen(dark_file));
    // RCLCPP_INFO(this->get_logger(), "Setting FFC dark field frame: %s", dark_file);
    // if (stat != XI_OK)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set FFC dark field frame.");
    //     xiCloseDevice(xi_handle_);
    //     xi_handle_ = nullptr;
    //     return false;
    // }

    // // Set the ffc flat field frame
    // char mid_file[] = "mid.tif";
    // stat = xiSetParamString(xi_handle_, XI_PRM_FFC_FLAT_FIELD_FILE_NAME, mid_file, strlen(mid_file));
    // RCLCPP_INFO(this->get_logger(), "Setting FFC flat field frame: %s", mid_file);
    // if (stat != XI_OK)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set FFC flat field frame.");
    //     xiCloseDevice(xi_handle_);
    //     xi_handle_ = nullptr;
    //     return false;
    // }

    // // Turn on the FFC
    // stat = xiSetParamInt(xi_handle_, XI_PRM_FFC, XI_ON);
    // if (stat != XI_OK)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to turn on FFC.");
    //     xiCloseDevice(xi_handle_);
    //     xi_handle_ = nullptr;
    //     return false;
    // }

    init_software_ffc();

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

void XimeaCaptureNode::init_software_ffc()
{
    std::string shading_dir = std::string(getenv("HOME")) + "/.local/share/xiCamTool/shading";
    
    std::filesystem::file_time_type latest_dark_time;
    std::filesystem::file_time_type latest_mid_time;

    // Search for the files in the shading directory
    for (const auto &entry : std::filesystem::directory_iterator(shading_dir))
    {
        if (entry.is_regular_file())
        {
            std::string file_path = entry.path().string();
            auto file_time = std::filesystem::last_write_time(entry);

            if (file_path.find("_dark.tif") != std::string::npos)
            {
                if (dark_file_.empty() || file_time > latest_dark_time)
                {
                    dark_file_ = file_path;
                    latest_dark_time = file_time;
                }
            }
            else if (file_path.find("_mid.tif") != std::string::npos)
            {
                if (mid_file_.empty() || file_time > latest_mid_time)
                {
                    mid_file_ = file_path;
                    latest_mid_time = file_time;
                }
            }
        }
    }

    if (dark_file_.empty() || mid_file_.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find FFC files in the shading directory.");
        return;
    }

    // Extract the filenames from the paths
    std::string dark_filename = std::filesystem::path(dark_file_).filename().string();
    std::string flat_filename = std::filesystem::path(mid_file_).filename().string();

    // Send the FFC filenames to the director
    auto ffc_msg = std_msgs::msg::String();
    ffc_msg.data = "FFC Dark Field File " + dark_filename;
    director_publisher_->publish(ffc_msg);
    ffc_msg.data = "FFC Flat Field File " + flat_filename;
    director_publisher_->publish(ffc_msg);

    // Load dark and mid field images
    dark_ = cv::imread(dark_file_, cv::IMREAD_GRAYSCALE);
    mid_ = cv::imread(mid_file_, cv::IMREAD_GRAYSCALE);

    if (dark_.empty() || mid_.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load FFC files.");
        return;
    }

    // Convert to float for computation
    dark_.convertTo(dark_, CV_32F);
    mid_.convertTo(mid_, CV_32F);

    // Compute mid - dark
    mid_dark_ = mid_ - dark_;

    // Mean of (mid - dark) for normalization
    mid_dark_mean_ = static_cast<float>(cv::mean(mid_dark_)[0]);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Software FFC initialized with dark: %s, mid: %s",
                dark_file_.c_str(), mid_file_.c_str());
}

cv::Mat XimeaCaptureNode::apply_software_ffc(cv::Mat &raw)
{
    // Ensure image sizes match
    if (raw.size() != dark_.size() || raw.size() != mid_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Image sizes do not match for FFC. Raw: %dx%d, Dark: %dx%d, Mid: %dx%d",
                     raw.cols, raw.rows, dark_.cols, dark_.rows, mid_.cols, mid_.rows);
        throw std::runtime_error("Calibration image size mismatch.");
    }

    raw.convertTo(raw, CV_32F);

    // Apply flat field correction formula (avoid division by zero)
    cv::Mat corrected_f = (raw - dark_).mul(mid_dark_mean_ / (mid_dark_ + 1e-6f));

    // Clip values to 0-255 and convert back to 8-bit
    cv::Mat clipped, corrected;
    cv::threshold(corrected_f, clipped, 255.0, 255.0, cv::THRESH_TRUNC);
    clipped.convertTo(corrected, CV_8U);

    return corrected;
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    XI_RETURN stat = xiGetImage(xi_handle_, 5000, &image);
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

    // Apply software FFC
    if (!dark_file_.empty() && !mid_file_.empty())
    {
        image_ = apply_software_ffc(image_);
    }

    // Create ROS Image message
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_).toImageMsg();

    // Publish the image
    image_publisher_->publish(*image_msg);

    auto completion_msg = std_msgs::msg::String();
    completion_msg.data = "XIMEA complete";
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