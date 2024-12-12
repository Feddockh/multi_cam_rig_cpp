#ifndef MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_
#define MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <m3api/xiApi.h> // Adjust the include path based on your installation

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <filesystem>

class XimeaCaptureNode : public rclcpp::Node
{
public:
    XimeaCaptureNode();

    ~XimeaCaptureNode();

private:
    void director_callback(const std_msgs::msg::String::SharedPtr msg);
    bool initialize_camera();
    void capture_image();
    void save_image();

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    bool save_images_;
    std::string save_dir_;
    int image_id_;

    // XIMEA Camera Handle
    HANDLE xi_handle_;
    bool camera_initialized_;

    // Camera Parameters
    int exposure_time_ = 10000; // in microseconds

    // Image storage
    cv::Mat image_;
};

#endif // MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_
