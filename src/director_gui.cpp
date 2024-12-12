#include "multi_cam_rig_cpp/director_gui.hpp"

DirectorGui::DirectorGui(int argc, char **argv)
    : QWidget(), rclcpp::Node("director_gui"), image_count_(1)
{

    // ROS 2 setup
    declare_parameter("director_topic", "/multi_cam_rig/director");
    declare_parameter("firefly_left_image_topic", "/flir_node/firefly_left/image_raw");
    declare_parameter("firefly_right_image_topic", "/flir_node/firefly_right/image_raw");
    declare_parameter("ximea_image_topic", "/multi_cam_rig/ximea/image");
    declare_parameter("zed_left_image_topic", "/multi_cam_rig/zed/left_image");
    declare_parameter("zed_right_image_topic", "/multi_cam_rig/zed/right_image");

    std::string director_topic = get_parameter("director_topic").as_string();
    std::string firefly_left_topic = get_parameter("firefly_left_image_topic").as_string();
    std::string firefly_right_topic = get_parameter("firefly_right_image_topic").as_string();
    std::string ximea_topic = get_parameter("ximea_image_topic").as_string();
    std::string zed_left_topic = get_parameter("zed_left_image_topic").as_string();
    std::string zed_right_topic = get_parameter("zed_right_image_topic").as_string();

    publisher_ = create_publisher<std_msgs::msg::String>(director_topic, 10);
    subscriber_ = create_subscription<std_msgs::msg::String>(
        director_topic, 10, std::bind(&DirectorGui::director_callback, this, std::placeholders::_1));

    // Subscribe to image topics
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        firefly_left_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, firefly_left_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        firefly_right_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, firefly_right_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        ximea_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, ximea_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        zed_left_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, zed_left_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        zed_right_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, zed_right_label_); }));

    // Qt GUI setup
    setWindowTitle("Director GUI with Multiple Image Displays");
    resize(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Main layout: Horizontal layout for left and right sections
    auto main_layout = new QHBoxLayout(this);

    // Left layout: Contains button and log area
    auto left_layout = new QVBoxLayout();
    left_layout->setSpacing(10); // Optional spacing between widgets

    // Add status label to the left layout
    status_label_ = new QLabel("Ready to capture images", this);
    status_label_->setAlignment(Qt::AlignCenter);
    left_layout->addWidget(status_label_);

    // Add capture button to the left layout
    auto button = new QPushButton("Capture Image", this);
    left_layout->addWidget(button);
    connect(button, &QPushButton::clicked, this, &DirectorGui::handle_button_click);

    // Add log area to the left layout
    log_area_ = new QTextEdit(this);
    log_area_->setReadOnly(true);
    log_area_->setMinimumHeight(100); // Optional: Set a minimum height
    left_layout->addWidget(log_area_);

    // Connect ROS 2 log messages to the GUI log area
    connect(this, &DirectorGui::newDirectorMessage, this, [this](const QString &m)
            { log_area_->append(m); });

    // Add the left layout to the main layout with 1/4 stretch
    main_layout->addLayout(left_layout, 1); // Stretch factor = 1

    // Right layout: Contains the multiple image displays
    auto right_layout = new QGridLayout();
    firefly_left_label_ = new QLabel(this);
    firefly_right_label_ = new QLabel(this);
    ximea_label_ = new QLabel(this);
    zed_left_label_ = new QLabel(this);
    zed_right_label_ = new QLabel(this);

    // Add image labels to the right layout
    for (QLabel *label : {firefly_left_label_, firefly_right_label_, ximea_label_, zed_left_label_, zed_right_label_})
    {
        label->setStyleSheet("border: 1px solid black;");
        label->setScaledContents(true);
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding); // Allow expanding
    }

    right_layout->addWidget(firefly_left_label_, 0, 0);
    right_layout->addWidget(firefly_right_label_, 0, 1);
    right_layout->addWidget(ximea_label_, 1, 0, 1, 2);
    right_layout->addWidget(zed_left_label_, 2, 0);
    right_layout->addWidget(zed_right_label_, 2, 1);

    // firefly_left_label_->setFixedSize(1080, 1080);
    // firefly_right_label_->setFixedSize(1080, 1080);
    // ximea_label_->setFixedSize(2048, 1088);
    // zed_left_label_->setFixedSize(1080, 1080);
    // zed_right_label_->setFixedSize(1080, 1080);

    // firefly_left_label_->setMaximumSize(500, 500);
    // firefly_right_label_->setMaximumSize(500, 500);
    // ximea_label_->setMaximumSize(800, 400);
    // zed_left_label_->setMaximumSize(500, 500);
    // zed_right_label_->setMaximumSize(500, 500);

    // Add the right layout to the main layout with 3/4 stretch
    main_layout->addLayout(right_layout, 3); // Stretch factor = 3
    setLayout(main_layout);

    // Apply stylesheet
    QString style_sheet = R"(
        QPushButton {
            background-color: #808080;
            color: white;
            font-size: 18px;
            padding: 10px;
            border-radius: 5px;
        }
        QPushButton:hover {
            background-color: #0078D7;
        }
        QPushButton:pressed {
            background-color: #005FA1;
        }
        QLabel {
            font-size: 16px;
            color: #444;
        }
        QTextEdit {
            background-color: #F9F9F9;
            border: 1px solid #DDD;
            font-size: 14px;
            color: #333;
        }
    )";
    setStyleSheet(style_sheet);
}

void DirectorGui::handle_button_click()
{
    auto message = std_msgs::msg::String();
    message.data = "capture " + std::to_string(image_count_++);
    publisher_->publish(message);

    status_label_->setText(QString("Triggered: %1").arg(QString::fromStdString(message.data)));
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
}

void DirectorGui::director_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Instead of directly appending to log_area_, emit the signal
    emit newDirectorMessage(QString::fromStdString(msg->data));
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

void DirectorGui::image_callback(const sensor_msgs::msg::Image::SharedPtr msg, QLabel *label)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        // Determine aspect ratio
        double aspect_ratio = static_cast<double>(cv_ptr->image.cols) / cv_ptr->image.rows;

        // Calculate label size based on window dimensions
        int max_width = (WINDOW_WIDTH * 3 / 4);
        int max_height = WINDOW_HEIGHT / 3;
        if (label != ximea_label_)
        {
            max_width = max_width / 2;
        }

        // Calculate scaled size maintaining aspect ratio
        int scaled_width = max_width;
        int scaled_height = static_cast<int>(scaled_width / aspect_ratio);
        if (scaled_height > max_height)
        {
            scaled_height = max_height;
            scaled_width = static_cast<int>(scaled_height * aspect_ratio);
        }

        QImage q_image_copy = QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_BGR888).copy();

        QMetaObject::invokeMethod(this, [label, q_image_copy, scaled_width, scaled_height]()
                                  { label->setPixmap(QPixmap::fromImage(q_image_copy).scaled(scaled_width, scaled_height, Qt::KeepAspectRatio, Qt::SmoothTransformation)); });
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto gui = std::make_shared<DirectorGui>(argc, argv);
    gui->show();

    // Spin in a separate thread
    std::thread spin_thread([gui]()
                            { rclcpp::spin(gui); });

    int ret = app.exec();

    rclcpp::shutdown();
    spin_thread.join();

    return ret;
}
