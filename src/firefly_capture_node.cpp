#include "multi_cam_rig_cpp/firefly_capture_node.hpp"

FireflyCaptureNode::FireflyCaptureNode()
    : Node("firefly_capture_node"),
      serial_port_name_("/dev/ttyUSB0"),
      io_context_(std::make_shared<drivers::common::IoContext>())
{
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
    if (msg->data.rfind("capture ", 0) == 0) // Message starts with "capture "
    {
        RCLCPP_INFO(this->get_logger(), "Received capture command: %s", msg->data.c_str());
        if (serial_port_ && serial_port_->is_open())
        {
            // Prepare the trigger message as a vector of bytes
            std::vector<uint8_t> trigger_message = {'t', '\n'};

            // Send the trigger message asynchronously
            serial_port_->send(trigger_message);
            // RCLCPP_INFO(this->get_logger(), "Sent serial trigger.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not initialized or open.");
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
        completion_msg.data = "Firefly Complete";
        director_publisher_->publish(completion_msg);
        RCLCPP_INFO(this->get_logger(), "Published Firefly images.");

        // Reset images
        left_image_.reset();
        right_image_.reset();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireflyCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
