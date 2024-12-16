#ifndef MULTI_CAM_RIG_CPP_FIREFLY_CAPTURE_NODE_HPP_
#define MULTI_CAM_RIG_CPP_FIREFLY_CAPTURE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <serial_driver/serial_driver.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <filesystem>

class FireflyCaptureNode : public rclcpp::Node
{
public:
    FireflyCaptureNode();
    ~FireflyCaptureNode();

private:
    void director_callback(const std_msgs::msg::String::SharedPtr msg);
    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void check_and_publish_completion();

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscriber_;

    int image_id_;

    // Serial communication
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
    std::string serial_port_name_;

    // Image storage
    sensor_msgs::msg::Image::SharedPtr left_image_;
    sensor_msgs::msg::Image::SharedPtr right_image_;
};

#endif // MULTI_CAM_RIG_CPP_FIREFLY_CAPTURE_NODE_HPP_
