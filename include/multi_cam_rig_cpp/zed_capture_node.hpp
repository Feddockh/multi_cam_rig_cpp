#ifndef MULTI_CAM_RIG_CPP_ZED_CAPTURE_NODE_HPP_
#define MULTI_CAM_RIG_CPP_ZED_CAPTURE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sl/Camera.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

class ZedCaptureNode : public rclcpp::Node
{
public:
    ZedCaptureNode();
    ~ZedCaptureNode();

private:
    void director_callback(const std_msgs::msg::String::SharedPtr msg);
    bool initialize_camera();
    void capture_image();
    void capture_imu_data();

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    int image_id_;

    sl::Camera zed_;
    bool camera_initialized_;

    // Image storage
    cv::Mat left_image_;
    cv::Mat right_image_;

    // IMU storage
    sl::SensorsData sensors_data_;
};

#endif // MULTI_CAM_RIG_CPP_ZED_CAPTURE_NODE_HPP_
