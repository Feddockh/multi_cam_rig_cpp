#include "multi_cam_rig_cpp/ximea_capture_node.hpp"

using namespace std::chrono_literals;

XimeaCaptureNode::XimeaCaptureNode()
    : Node("ximea_capture_node"), xi_handle_(nullptr), camera_initialized_(false)
{
    // Declare and get parameters
    declare_parameter("data_dir", "~/tmp");
    declare_parameter<std::string>("director_topic", "/multi_cam_rig/director");
    declare_parameter<std::string>("ximea_image_topic", "/multi_cam_rig/ximea/image");

    std::string director_topic = get_parameter("director_topic").as_string();
    std::string image_topic = get_parameter("ximea_image_topic").as_string();

    std::string data_dir = get_parameter("data_dir").as_string();
    if (!data_dir.empty() && data_dir[0] == '~')
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
        {
            data_dir.replace(0, 1, home);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "HOME environment variable not set.");
        }
    }
    data_dir_ = data_dir;

    // Create the FFC directory inside the data directory.
    ffc_dir_ = data_dir_ + "/ffc";
    if (!std::filesystem::exists(ffc_dir_))
    {
        std::filesystem::create_directories(ffc_dir_);
    }

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

    // Seems to make worse?
    // stat = xiSetParamInt(xi_handle_, XI_PRM_OUTPUT_DATA_BIT_DEPTH, XI_RAW8);
    // if (stat != XI_OK) {
    //     RCLCPP_WARN(this->get_logger(), "Failed to set output data bit depth.");
    // }

    // Increase the internal buffer queue size
    stat = xiSetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE, 10);
    if (stat != XI_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set buffers queue size.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // This parameter tells the driver how many buffers to commit to the transport layer (e.g., USB)
    // Increasing this value may improve USB transport performance.
    stat = xiSetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, 20);
    if (stat != XI_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set acq_transport_buffer_commit.");
    }

    // Enable data packing to reduce data size (I think this helps, not sure)
    stat = xiSetParamInt(xi_handle_, XI_PRM_OUTPUT_DATA_PACKING, XI_ON);
    if (stat != XI_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to enable output data packing.");
    }

    // NEW: Enable recent frame mode so that xiGetImage returns the most recent frame
    stat = xiSetParamInt(xi_handle_, XI_PRM_RECENT_FRAME, 1);
    if (stat != XI_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set recent_frame mode.");
    }

    // Set auto exposure/gain to off
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG, XI_OFF);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set auto exposure/gain.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Set exposure time (in microseconds)
    stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, exposure_time_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Set the gain
    stat = xiSetParamFloat(xi_handle_, XI_PRM_GAIN, gain_);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain.");
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

    // Set image format
    xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8);
    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set image format.");
        xiCloseDevice(xi_handle_);
        xi_handle_ = nullptr;
        return false;
    }

    // Initialize software FFC
    if (!init_software_ffc())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize software FFC.");
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

bool XimeaCaptureNode::init_software_ffc()
{
    std::filesystem::file_time_type latest_dark_time;
    std::filesystem::file_time_type latest_mid_time;

    // Search for the files in the shading directory
    for (const auto &entry : std::filesystem::directory_iterator(ffc_dir_))
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
        return false;
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
        return false;
    }

    // Convert to float for computation
    dark_.convertTo(dark_, CV_32F);
    mid_.convertTo(mid_, CV_32F);

    // Compute mid - dark
    mid_dark_ = mid_ - dark_;

    // Mean of (mid - dark) for normalization
    mid_dark_mean_ = static_cast<float>(cv::mean(mid_dark_)[0]);
    FFC_ = mid_dark_mean_ / (mid_dark_ + 1e-6f);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Software FFC initialized with dark: %s, mid: %s",
                dark_file_.c_str(), mid_file_.c_str());

    return true;
}

cv::Mat XimeaCaptureNode::apply_software_ffc(cv::Mat &raw)
{
    // Ensure image sizes match
    if (raw.size() != dark_.size() || raw.size() != mid_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Image sizes do not match for FFC. Raw: %dx%d, Dark: %dx%d, Mid: %dx%d",
                     raw.cols, raw.rows, dark_.cols, dark_.rows, mid_.cols, mid_.rows);
        // throw std::runtime_error("Calibration image size mismatch.");
        return raw;
    }

    raw.convertTo(raw, CV_32F);

    // Apply flat field correction formula (avoid division by zero)
    cv::Mat corrected_f = (raw - dark_).mul(FFC_);

    // Clip values to 0-255 and convert back to 8-bit
    cv::Mat clipped, corrected;
    cv::threshold(corrected_f, clipped, 255.0, 255.0, cv::THRESH_TRUNC);
    clipped.convertTo(corrected, CV_8U);

    return corrected;
}

void XimeaCaptureNode::director_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Check if the message starts with "capture "
    if (msg->data.rfind("Capture ", 0) == 0)
    {
        // Extract the image ID from the message
        RCLCPP_INFO(this->get_logger(), "Received capture command: %s", msg->data.c_str());
        image_id_ = std::stoi(msg->data.substr(8));

        // Capture the image
        if (camera_initialized_)
        {
            cv::Mat img = capture_calibrated_image();
            publish_image(img);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "XIMEA camera not initialized, cannot capture image.");
        }
    }
    else if (msg->data.rfind("Ximea gain: ", 0) == 0)
    {
        try
        {
            float new_gain = std::stof(msg->data.substr(12));
            if (new_gain != gain_)
            {
                gain_ = new_gain;
                XI_RETURN stat = xiSetParamFloat(xi_handle_, XI_PRM_GAIN, gain_);
                if (stat != XI_OK)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set gain to %f", gain_);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Gain updated to %f", gain_);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Gain already set to %f", gain_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing gain: %s", e.what());
        }
    }
    else if (msg->data.rfind("Ximea exposure: ", 0) == 0)
    {
        try
        {
            int new_exposure = std::stoi(msg->data.substr(16));
            if (new_exposure != exposure_time_)
            {
                exposure_time_ = new_exposure;
                XI_RETURN stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, exposure_time_);
                if (stat != XI_OK)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time to %d", exposure_time_);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Exposure time updated to %d", exposure_time_);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Exposure time already set to %d", exposure_time_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error parsing exposure time: %s", e.what());
        }
    }
    else if (msg->data == "FFC calibrate")
    {
        RCLCPP_INFO(this->get_logger(), "Received FFC calibrate command.");
        calibrate_ffc();
    }
    else if (msg->data == "Dark calibrate")
    {
        RCLCPP_INFO(this->get_logger(), "Received Dark calibrate command.");
        calibrate_dark();
    }
}

cv::Mat XimeaCaptureNode::capture_raw_image()
{
    if (!camera_initialized_ || xi_handle_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Camera is not initialized.");
        return cv::Mat();
    }

    // NEW: Start timing before calling xiGetImage.
    auto start = std::chrono::steady_clock::now();

    XI_IMG image;
    memset(&image, 0, sizeof(image));
    image.size = sizeof(XI_IMG);

    XI_RETURN stat = xiGetImage(xi_handle_, 1000, &image);

    // NEW: End timing after xiGetImage.
    auto get_image_end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "xiGetImage took %ld microseconds",
                 std::chrono::duration_cast<std::chrono::microseconds>(get_image_end - start).count());

    if (stat != XI_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get image.");
        return cv::Mat();
    }

    if (image.bp == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Image buffer is null.");
        return cv::Mat();
    }

    RCLCPP_INFO(this->get_logger(), "Image Width: %d, Height: %d, Format: %d", image.width, image.height, image.frm);

    cv::Mat cvImageMono(image.height, image.width, CV_8UC1, image.bp);

    // NEW: End timing after conversion.
    auto conversion_end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Image conversion took %ld microseconds",
                 std::chrono::duration_cast<std::chrono::microseconds>(conversion_end - get_image_end).count());

    return cvImageMono.clone();
}

cv::Mat XimeaCaptureNode::capture_calibrated_image()
{
    // NEW: Start overall timing
    auto start = std::chrono::steady_clock::now();

    cv::Mat raw = capture_raw_image();

    // If the raw image is empty, return an empty image
    if (raw.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture raw image.");
        return cv::Mat();
    }

    // If raw image size is 0 by 0, return an empty image
    if (raw.size().area() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Raw image size is 0 by 0.");
        return cv::Mat();
    }   

    // NEW: Log the time to capture the raw image
    auto raw_capture_end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "capture_raw_image took %ld microseconds",
                 std::chrono::duration_cast<std::chrono::microseconds>(raw_capture_end - start).count());

    // Apply software FFC
    cv::Mat img;
    if (!dark_file_.empty() && !mid_file_.empty())
    {
        img = apply_software_ffc(raw);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "FFC files not found, capturing uncalibrated image.");
        img = raw;
    }

    // NEW: End overall timing for capture_calibrated_image
    auto end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "capture_calibrated_image total took %ld microseconds",
                 std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());

    return img;
}

void XimeaCaptureNode::calibrate_ffc()
{
    cv::Mat sum;
    const int count = 5;
    for (int i = 0; i < count; i++)
    {
        cv::Mat img = capture_raw_image();
        if (img.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture calibration image %d", i);
            return;
        }
        cv::Mat imgFloat;
        img.convertTo(imgFloat, CV_32F);
        if (i == 0)
            sum = imgFloat;
        else
            sum += imgFloat;
        std::this_thread::sleep_for(100ms);
    }
    cv::Mat avg;
    sum.convertTo(avg, CV_32F, 1.0 / count);
    avg.convertTo(avg, CV_8U);

    // Generate a new filename based on the current time.
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now_c));
    std::string new_file = std::string(buffer) + "_mid.tif";
    std::string new_file_path = ffc_dir_ + '/' + new_file;

    if (!cv::imwrite(new_file_path, avg))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write FFC calibration image to file %s", new_file.c_str());
        return;
    }

    mid_file_ = new_file;
    mid_ = avg;
    if (!dark_.empty())
    {
        mid_.convertTo(mid_, CV_32F);
        dark_.convertTo(dark_, CV_32F);
        mid_dark_ = mid_ - dark_;
        mid_dark_mean_ = static_cast<float>(cv::mean(mid_dark_)[0]);
        FFC_ = mid_dark_mean_ / (mid_dark_ + 1e-6f);
    }
    auto msg = std_msgs::msg::String();
    msg.data = "FFC calibration complete: " + new_file;
    director_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "FFC calibration complete, saved new mid file: %s", new_file.c_str());
}

void XimeaCaptureNode::calibrate_dark()
{
    cv::Mat sum;
    const int count = 5;
    for (int i = 0; i < count; i++)
    {
        cv::Mat img = capture_raw_image();
        if (img.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture calibration image %d", i);
            return;
        }
        cv::Mat imgFloat;
        img.convertTo(imgFloat, CV_32F);
        if (i == 0)
            sum = imgFloat;
        else
            sum += imgFloat;
        std::this_thread::sleep_for(100ms);
    }
    cv::Mat avg;
    sum.convertTo(avg, CV_32F, 1.0 / count);
    avg.convertTo(avg, CV_8U);

    // Generate a new filename based on the current time.
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now_c));
    std::string new_file = std::string(buffer) + "_dark.tif";
    std::string new_file_path = ffc_dir_ + '/' + new_file;

    if (!cv::imwrite(new_file_path, avg))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write dark calibration image to file %s", new_file.c_str());
        return;
    }

    dark_file_ = new_file;
    dark_ = avg;
    if (!mid_file_.empty() && !mid_.empty())
    {
        mid_.convertTo(mid_, CV_32F);
        dark_.convertTo(dark_, CV_32F);
        mid_dark_ = mid_ - dark_;
        mid_dark_mean_ = static_cast<float>(cv::mean(mid_dark_)[0]);
        FFC_ = mid_dark_mean_ / (mid_dark_ + 1e-6f);
    }
    auto msg = std_msgs::msg::String();
    msg.data = "Dark calibration complete: " + new_file;
    director_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Dark calibration complete, saved new dark file: %s", new_file.c_str());
}

void XimeaCaptureNode::publish_image(cv::Mat &image)
{
    // NEW: Start timing before conversion.
    auto start = std::chrono::steady_clock::now();

    // Create ROS Image message
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();

    // Publish the image
    image_publisher_->publish(*image_msg);

    // NEW: End timing after publishing.
    auto end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "publish_image took %ld microseconds",
                 std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());

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