#ifndef MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_
#define MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <m3api/xiApi.h>

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
    void init_software_ffc();
    cv::Mat apply_software_ffc(cv::Mat &raw);

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    int image_id_;

    // XIMEA Camera Handle
    HANDLE xi_handle_;
    bool camera_initialized_;

    // Image storage
    cv::Mat image_;

    // FFC files
    std::string dark_file_;
    std::string mid_file_;
    cv::Mat dark_;
    cv::Mat mid_;
    cv::Mat mid_dark_;
    float mid_dark_mean_;
};

#endif // MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_
