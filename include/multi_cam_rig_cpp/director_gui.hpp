#ifndef MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_
#define MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <QTimer>
#include <QCloseEvent>
#include <QSlider>
#include <thread>
#include <filesystem>
#include <QScreen>
#include <QGuiApplication>
#include <chrono>
#include <QtConcurrent/QtConcurrent>

class DirectorGui : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    DirectorGui(int argc, char **argv);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void director_callback(const std_msgs::msg::String::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, QLabel *label);

    std::string director_topic_;
    std::string firefly_left_topic_;
    std::string firefly_right_topic_;
    std::string ximea_topic_;
    std::string zed_left_topic_;
    std::string zed_right_topic_;
    std::string zed_imu_topic_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    int image_count_;
    std::string data_dir_;
    int rosbag_pid_ = -1;

    int window_height_;
    int window_width_;

    // Left column widgets (capture controls and status)
    QPushButton *capture_button_;
    QPushButton *record_button_;
    QPushButton *rosbag_button_;
    QLabel *status_label_;
    QTextEdit *log_area_;

    // Middle column widgets (exposure controls)
    QLabel *firefly_exposure_label_;
    QSlider *firefly_exposure_slider_;
    QLabel *firefly_exposure_value_label_;
    QLabel *ximea_exposure_label_;
    QSlider *ximea_exposure_slider_;
    QLabel *ximea_exposure_value_label_;
    QLabel *zed_exposure_label_;
    QSlider *zed_exposure_slider_;
    QLabel *zed_exposure_value_label_;

    QPushButton *update_settings_button_;

    // Right column widgets (image displays)
    QLabel *firefly_left_label_;
    QLabel *firefly_right_label_;
    QLabel *ximea_label_;
    QLabel *zed_left_label_;
    QLabel *zed_right_label_;

    // New calibration buttons
    QPushButton *ffc_calibrate_button_;
    QPushButton *dark_calibrate_button_;

    QTimer *timer_;
    bool recording_ = false;

private slots:
    void handle_capture_button_click();
    void handle_record_button_click();
    void handle_rosbag_button_click();

    void handle_update_settings();

    void handle_ffc_calibrate_button_click();
    void handle_dark_calibrate_button_click();

signals:
    void newDirectorMessage(QString msg);
};

#endif // MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_
