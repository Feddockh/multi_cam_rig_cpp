#include "multi_cam_rig_cpp/director_gui.hpp"

DirectorGui::DirectorGui(int argc, char **argv)
    : QWidget(), rclcpp::Node("director_gui"), image_count_(1)
{
    // Declare and get parameters
    declare_parameter("data_dir", "~/tmp");
    declare_parameter("director_topic", "/multi_cam_rig/director");
    declare_parameter("firefly_left_image_topic", "/flir_node/firefly_left/image_raw");
    declare_parameter("firefly_right_image_topic", "/flir_node/firefly_right/image_raw");
    declare_parameter("ximea_image_topic", "/multi_cam_rig/ximea/image");
    declare_parameter("zed_left_image_topic", "/multi_cam_rig/zed/left_image");
    declare_parameter("zed_right_image_topic", "/multi_cam_rig/zed/right_image");
    declare_parameter("zed_imu_topic", "/multi_cam_rig/zed/imu");

    data_dir_ = get_parameter("data_dir").as_string();
    director_topic_ = get_parameter("director_topic").as_string();
    firefly_left_topic_ = get_parameter("firefly_left_image_topic").as_string();
    firefly_right_topic_ = get_parameter("firefly_right_image_topic").as_string();
    ximea_topic_ = get_parameter("ximea_image_topic").as_string();
    zed_left_topic_ = get_parameter("zed_left_image_topic").as_string();
    zed_right_topic_ = get_parameter("zed_right_image_topic").as_string();
    zed_imu_topic_ = get_parameter("zed_imu_topic").as_string();

    // Set up director's publisher and subscriber
    publisher_ = create_publisher<std_msgs::msg::String>(director_topic_, 10);
    subscriber_ = create_subscription<std_msgs::msg::String>(
        director_topic_, 10, std::bind(&DirectorGui::director_callback, this, std::placeholders::_1));

    // Subscribe to image topics
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        firefly_left_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, firefly_left_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        firefly_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, firefly_right_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        ximea_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, ximea_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        zed_left_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, zed_left_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        zed_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, zed_right_label_); }));

    // Subscribe to IMU topic
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        zed_imu_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { RCLCPP_INFO(this->get_logger(), "Received IMU data."); });

    // Qt GUI setup
    setWindowTitle("Director GUI with Multiple Image Displays");

    // Get the screen dimensions
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    window_width_ = screenGeometry.width();
    window_height_ = screenGeometry.height();
    resize(window_width_, window_height_);

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
    capture_button_ = new QPushButton("Capture Image", this);
    capture_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    left_layout->addWidget(capture_button_, 1);
    connect(capture_button_, &QPushButton::clicked, this, &DirectorGui::handle_capture_button_click);

    // Add record button to the left layout
    record_button_ = new QPushButton("Record Video", this);
    record_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    left_layout->addWidget(record_button_, 1);
    connect(record_button_, &QPushButton::clicked, this, &DirectorGui::handle_record_button_click);

    // Add log area to the left layout
    log_area_ = new QTextEdit(this);
    log_area_->setReadOnly(true);
    log_area_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    left_layout->addWidget(log_area_, 1);

    // Add a button to start recording a rosbag
    rosbag_button_ = new QPushButton("Start Rosbag", this);
    rosbag_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    left_layout->addWidget(rosbag_button_, 1);
    connect(rosbag_button_, &QPushButton::clicked, this, &DirectorGui::handle_rosbag_button_click);

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

void DirectorGui::handle_capture_button_click()
{
    auto message = std_msgs::msg::String();
    message.data = "capture " + std::to_string(image_count_++);
    publisher_->publish(message);

    status_label_->setText(QString("Triggered: %1").arg(QString::fromStdString(message.data)));
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
}

void DirectorGui::handle_record_button_click()
{
    if (recording_)
    {
        timer_->stop();
        recording_ = false;
        auto message = std_msgs::msg::String();
        message.data = "Recording stopped";
        publisher_->publish(message);
        status_label_->setText("Recording stopped");
        RCLCPP_INFO(this->get_logger(), "Recording stopped");
        record_button_->setStyleSheet("");
    }
    else
    {
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, [this]()
                {
                    auto message = std_msgs::msg::String();
                    message.data = "capture " + std::to_string(image_count_++);
                    publisher_->publish(message);

                    status_label_->setText(QString("Recording: %1").arg(QString::fromStdString(message.data)));
                    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str()); });
        timer_->start(1000); // 1 second interval
        recording_ = true;

        auto message = std_msgs::msg::String();
        message.data = "Recording started";
        publisher_->publish(message);
        status_label_->setText("Recording started");
        RCLCPP_INFO(this->get_logger(), "Recording started");
        record_button_->setStyleSheet("background-color: blue;");
    }
}

void DirectorGui::handle_rosbag_button_click()
{
    if (rosbag_pid_ != -1)
    {
        kill(rosbag_pid_, SIGINT);
        auto message = std_msgs::msg::String();
        message.data = "Stopped recording rosbag with PID: " + std::to_string(rosbag_pid_);
        publisher_->publish(message);
        status_label_->setText("Stopped recording rosbag");
        RCLCPP_INFO(this->get_logger(), "Stopped recording rosbag with PID: %d", rosbag_pid_);
        rosbag_pid_ = -1;
        rosbag_button_->setStyleSheet("");
    }
    else
    {
        // Get current date and time
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        // Create a directory with the current date and time
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string dir_name = data_dir_ + "/rosbag_" + ss.str();
        
        // Start the rosbag recording (in the background)
        std::string cmd = "ros2 bag record -o " + dir_name + " " 
            + director_topic_ + " "
            + firefly_left_topic_ + " " 
            + firefly_right_topic_ + " " 
            + ximea_topic_ + " " 
            + zed_left_topic_ + " " 
            + zed_right_topic_ + " " 
            + zed_imu_topic_ + "> /dev/null 2>&1 & echo $!";

        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe)
        {
            char buffer[128];
            std::string result = "";
            while (!feof(pipe))
            {
                if (fgets(buffer, 128, pipe) != NULL)
                    result += buffer;
            }
            pclose(pipe);
            rosbag_pid_ = std::stoi(result);
        }

        auto message = std_msgs::msg::String();
        message.data = "Started recording rosbag with PID: " + std::to_string(rosbag_pid_);
        publisher_->publish(message);
        status_label_->setText("Started recording rosbag");
        RCLCPP_INFO(this->get_logger(), "Started recording rosbag with PID: %d", rosbag_pid_);
        rosbag_button_->setStyleSheet("background-color: red;");
    }
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
        int max_width = (window_width_ * 3 / 4);
        int max_height = window_height_ / 3;
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

void DirectorGui::closeEvent(QCloseEvent *event)
{
    RCLCPP_INFO(this->get_logger(), "GUI window is closing. Initiating shutdown.");

    // Stop recording if running
    if (recording_)
    {
        timer_->stop();
    }

    // Shutdown ROS 2 node
    rclcpp::shutdown();

    // Close the GUI
    event->accept();
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto gui = std::make_shared<DirectorGui>(argc, argv);
    gui->show();

    // Start ROS spinning in another thread
    std::thread spin_thread([gui]()
                            {
        rclcpp::spin(gui);
        // Once spin() returns (due to shutdown), we instruct Qt to quit
        QMetaObject::invokeMethod(QApplication::instance(), "quit", Qt::QueuedConnection); });

    // Run the Qt event loop
    int ret = app.exec();

    // Cleanup after Qt loop ends
    rclcpp::shutdown();
    spin_thread.join();

    return ret;
}
