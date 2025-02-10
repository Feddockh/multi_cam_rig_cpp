#include "multi_cam_rig_cpp/firefly_capture_node.hpp"

FireflyCaptureNode::FireflyCaptureNode()
    : Node("firefly_capture_node"),
      serial_port_name_("/dev/ttyUSB0"),
      io_context_(std::make_shared<drivers::common::IoContext>())
{
    // Declare and get parameters
    declare_parameter("director_topic", "/multi_cam_rig/director");
    declare_parameter("left_image_topic", "/flir_node/firefly_left/image_raw");
    declare_parameter("right_image_topic", "/flir_node/firefly_right/image_raw");

    std::string director_topic = get_parameter("director_topic").as_string();
    std::string left_image_topic = get_parameter("left_image_topic").as_string();
    std::string right_image_topic = get_parameter("right_image_topic").as_string();

    // Initialize publishers and subscribers
    director_publisher_ = create_publisher<std_msgs::msg::String>(director_topic, 10);
    director_subscriber_ = create_subscription<std_msgs::msg::String>(
        director_topic, 10, std::bind(&FireflyCaptureNode::director_callback, this, std::placeholders::_1));
    left_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        left_image_topic, 10, std::bind(&FireflyCaptureNode::left_image_callback, this, std::placeholders::_1));
    right_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        right_image_topic, 10, std::bind(&FireflyCaptureNode::right_image_callback, this, std::placeholders::_1));

    // Configure and initialize the serial port
    drivers::serial_driver::SerialPortConfig config(9600, drivers::serial_driver::FlowControl::NONE,
                                                    drivers::serial_driver::Parity::NONE,
                                                    drivers::serial_driver::StopBits::ONE);

    try
    {
        serial_port_ = std::make_shared<drivers::serial_driver::SerialPort>(*io_context_, serial_port_name_, config);
        serial_port_->open();
        RCLCPP_INFO(this->get_logger(), "Serial port initialized on %s", serial_port_name_.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", e.what());
    }

    // Set the flash duration
    set_flash_duration(flash_duration_);
    set_flash_frequency(flash_frequency_);
}

FireflyCaptureNode::~FireflyCaptureNode()
{
    if (serial_port_ && serial_port_->is_open())
    {
        serial_port_->close();
        RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }
}

void FireflyCaptureNode::director_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data.rfind("Capture ", 0) == 0) // Message starts with "capture "
    {
        // Extract the image ID from the message
        RCLCPP_INFO(this->get_logger(), "Received capture command: %s", msg->data.c_str());
        image_id_ = std::stoi(msg->data.substr(8));

        // Check if the serial port is open
        if (serial_port_ && serial_port_->is_open())
        {
            // Prepare the trigger message as a vector of bytes
            std::vector<uint8_t> trigger_message = {'t', '\n'};

            // Send the trigger message asynchronously
            serial_port_->send(trigger_message);
            // RCLCPP_INFO(this->get_logger(), "Sent serial trigger.");

            // // Read the response (not working)
            // std::vector<uint8_t> response;
            // serial_port_->receive(response);
            // std::string response_str(response.begin(), response.end());
            // RCLCPP_INFO(this->get_logger(), "Received response: %s", response_str.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not initialized or open.");
        }
    }
    else if (msg->data.rfind("Flash duration: ", 0) == 0)
    {
        int new_duration = std::stoi(msg->data.substr(16));
        set_flash_duration(new_duration);
    }
    else if (msg->data.rfind("Firefly exposure: ", 0) == 0) {
        try {
            int new_exposure = std::stoi(msg->data.substr(18)); // adjust the substring offset as needed
            if (!set_exposure_time(new_exposure)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update firefly exposure time.");
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing exposure time: %s", e.what());
        }
    }
}

void FireflyCaptureNode::left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    left_image_ = msg;
    check_and_publish_completion();
}

void FireflyCaptureNode::right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    right_image_ = msg;
    check_and_publish_completion();
}

void FireflyCaptureNode::check_and_publish_completion()
{
    if (left_image_ && right_image_)
    {
        auto completion_msg = std_msgs::msg::String();
        completion_msg.data = "Firefly complete";
        director_publisher_->publish(completion_msg);
        RCLCPP_INFO(this->get_logger(), "Published Firefly images.");

        // Reset images
        left_image_.reset();
        right_image_.reset();
    }
}

bool FireflyCaptureNode::set_flash_duration(int duration)
{
    if (serial_port_ && serial_port_->is_open())
    {
        // Prepare to set the flash duration
        std::vector<uint8_t> flash_set_message = {'f', '\n'};
        serial_port_->send(flash_set_message);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Set the flash duration
        std::string duration_str = std::to_string(duration) + '\n';
        std::vector<uint8_t> flash_duration_message(duration_str.begin(), duration_str.end());
        serial_port_->send(flash_duration_message);

        RCLCPP_INFO(this->get_logger(), "Set flash duration to: %d", duration);

        // Update the flash duration
        flash_duration_ = duration;

        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port not initialized or open.");
        return false;
    }
}

bool FireflyCaptureNode::set_flash_frequency(int frequency)
{
    if (serial_port_ && serial_port_->is_open())
    {
        // Prepare to set the flash frequency
        std::vector<uint8_t> flash_set_message = {'c', '\n'};
        serial_port_->send(flash_set_message);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Set the flash frequency
        std::string frequency_str = std::to_string(frequency) + '\n';
        std::vector<uint8_t> flash_frequency_message(frequency_str.begin(), frequency_str.end());
        serial_port_->send(flash_frequency_message);

        RCLCPP_INFO(this->get_logger(), "Set flash frequency to: %d", frequency);

        // Update the flash frequency
        flash_frequency_ = frequency;

        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port not initialized or open.");
        return false;
    }
}

bool FireflyCaptureNode::set_exposure_time(int time)
{
    // Create a synchronous parameter client for the node that owns the firefly parameters.
    auto parameter_client = rclcpp::SyncParametersClient::make_shared(this, "/flir_node");

    // Wait for the parameter service to become available (2 seconds timeout).
    if (!parameter_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for /flir_node not available.");
        return false;
    }

    // Create the parameters to update for both left and right firefly cameras.
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("firefly_left.exposure_time", time));
    parameters.push_back(rclcpp::Parameter("firefly_right.exposure_time", time));

    // Set the parameters and check the results.
    auto results = parameter_client->set_parameters(parameters);
    bool success = true;
    for (const auto &result : results) {
        if (!result.successful) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time: %s", result.reason.c_str());
            success = false;
        }
    }
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Exposure time updated to %d for firefly left and right.", time);
    }
    return success;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireflyCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
