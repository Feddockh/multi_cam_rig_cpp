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
#include <sys/wait.h>

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
    void stop_recording();

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
    std::string current_rosbag_dir_;

    int window_height_;
    int window_width_;

    // Left column widgets (capture controls and status)
    QPushButton *capture_button_;
    QPushButton *record_button_;
    QPushButton *rosbag_button_;
    QLabel *status_label_;
    QTextEdit *log_area_;

    // Const values for exposure and gain
    const int flash_duration_min_ = 0;
    const int flash_duration_max_ = 500;
    const int firefly_exposure_min_ = 30;
    // const int firefly_exposure_max_ = 200;
    const int firefly_exposure_max_ = 10000;
    const float ximea_gain_min_ = -1.5;
    const float ximea_gain_max_ = 6.0;
    const int ximea_exposure_min_ = 1;
    // const int ximea_exposure_max_ = 10000;
    const int ximea_exposure_max_ = 100000;

    // Middle column widgets (exposure controls)
    int flash_duration_ = 100; // 0 - 500 us
    QLabel *flash_duration_label_;
    QSlider *flash_duration_slider_;
    QLabel *flash_duration_value_label_;
    int firefly_exposure_ = 100; // 29 - 30000014 us
    QLabel *firefly_exposure_label_;
    QSlider *firefly_exposure_slider_;
    QLabel *firefly_exposure_value_label_;
    float ximea_gain_ = 0.0; // -1.5 - 6.0 dB
    QLabel *ximea_gain_label_;
    QSlider *ximea_gain_slider_;
    QLabel *ximea_gain_value_label_;
    int ximea_exposure_ = 100; // 1 - 1000000 us
    QLabel *ximea_exposure_label_;
    QSlider *ximea_exposure_slider_;
    QLabel *ximea_exposure_value_label_;

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
