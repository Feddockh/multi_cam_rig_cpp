#ifndef MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_
#define MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QApplication>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <thread>
#include <filesystem>

class DirectorGui : public QWidget, public rclcpp::Node
{
    Q_OBJECT

public:
    DirectorGui(int argc, char **argv);

private:
    void director_callback(const std_msgs::msg::String::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg, QLabel *label);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;

    int image_count_;
    bool save_images_;
    std::string save_dir_;

    static constexpr int WINDOW_HEIGHT = 800;
    static constexpr int WINDOW_WIDTH = 1200;

    QLabel *status_label_;
    QTextEdit *log_area_;

    QLabel *firefly_left_label_;
    QLabel *firefly_right_label_;
    QLabel *ximea_label_;
    QLabel *zed_left_label_;
    QLabel *zed_right_label_;

private slots:
    void handle_button_click();

signals:
    void newDirectorMessage(QString msg);
};

#endif // MULTI_CAM_RIG_CPP_DIRECTOR_GUI_HPP_
