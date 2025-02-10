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
#include "rclcpp/parameter_client.hpp"

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
    bool set_flash_duration(int duration);
    bool set_flash_frequency(int frequency);
    bool set_exposure_time(int time);

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscriber_;

    int image_id_;
    int flash_duration_ = 100; // 50 - 300 us
    int exposure_time_ = 1000; // 100 - 300,000 us
    int flash_frequency_ = 20; // 1 - 20HZ

    // Serial communication
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
    std::string serial_port_name_;

    // Image storage
    sensor_msgs::msg::Image::SharedPtr left_image_;
    sensor_msgs::msg::Image::SharedPtr right_image_;
};

#endif // MULTI_CAM_RIG_CPP_FIREFLY_CAPTURE_NODE_HPP_
