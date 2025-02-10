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
    bool init_software_ffc();
    cv::Mat apply_software_ffc(cv::Mat &raw);
    cv::Mat capture_raw_image();
    cv::Mat capture_calibrated_image();
    void calibrate_ffc();
    void calibrate_dark();
    void publish_image(cv::Mat &image);

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr director_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr director_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    int image_id_;
    float gain_ = 0.0; // -1.5 - 6.0 dB
    int exposure_time_ = 100000; // 500 - 300,000 us

    // XIMEA Camera Handle
    HANDLE xi_handle_;
    bool camera_initialized_;

    // std::string FFC_dir_ = std::string(getenv("HOME")) + "/.local/share/xiCamTool/shading";
    std::string data_dir_;
    std::string ffc_dir_;

    // FFC files
    std::string mid_file_;
    std::string dark_file_;
    cv::Mat dark_;
    cv::Mat mid_;
    cv::Mat mid_dark_;
    cv::Mat FFC_;
    float mid_dark_mean_;
};

#endif // MULTI_CAM_RIG_CPP_XIMEA_CAPTURE_NODE_HPP_
