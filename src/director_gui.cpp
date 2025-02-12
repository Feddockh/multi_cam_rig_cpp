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
    // image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
    //     firefly_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
    //     { image_callback(msg, firefly_right_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        ximea_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, ximea_label_); }));
    image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
        zed_left_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { image_callback(msg, zed_left_label_); }));
    // image_subscribers_.emplace_back(create_subscription<sensor_msgs::msg::Image>(
    //     zed_right_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg)
    //     { image_callback(msg, zed_right_label_); }));

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

    // --- Main Layout: Three Equal Columns ---
    auto main_layout = new QHBoxLayout(this);

    // --- Left Column: Capture/Record/Output/Rosbag ---
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

    // Add left layout with 1/4 stretch to the main layout
    main_layout->addLayout(left_layout, 1); // Stretch factor = 1

    // --- Middle Column: Exposure Controls (Using Sliders) ---
    auto middle_layout = new QVBoxLayout();
    middle_layout->setSpacing(10);
    middle_layout->setAlignment(Qt::AlignCenter); // Center the controls vertically

    // Flash Duration Control
    flash_duration_label_ = new QLabel(QString("Flash Duration (%1 - %2 us)").arg(flash_duration_min_).arg(flash_duration_max_), this);
    flash_duration_label_->setAlignment(Qt::AlignCenter);
    middle_layout->addWidget(flash_duration_label_);

    auto flash_duration_layout = new QHBoxLayout();
    flash_duration_slider_ = new QSlider(Qt::Horizontal, this);
    flash_duration_slider_->setRange(flash_duration_min_, flash_duration_max_);
    flash_duration_slider_->setMinimumHeight(80);
    flash_duration_slider_->setTickPosition(QSlider::TicksBelow);
    flash_duration_slider_->setTickInterval(10);
    flash_duration_slider_->setValue(flash_duration_);
    flash_duration_layout->addWidget(flash_duration_slider_);

    flash_duration_value_label_ = new QLabel(QString::number(flash_duration_slider_->value()), this);
    flash_duration_value_label_->setAlignment(Qt::AlignCenter);
    flash_duration_layout->addWidget(flash_duration_value_label_);
    middle_layout->addLayout(flash_duration_layout);

    connect(flash_duration_slider_, &QSlider::valueChanged, this,
            [this](int value) { flash_duration_value_label_->setText(QString::number(value)); });

    // Firefly Exposure Control
    firefly_exposure_label_ = new QLabel(
    QString("Firefly Exposure (%1 - %2 us)")
        .arg(firefly_exposure_min_)
        .arg(firefly_exposure_max_),
    this);
    firefly_exposure_label_->setAlignment(Qt::AlignCenter);
    middle_layout->addWidget(firefly_exposure_label_);

    // Create a horizontal layout for the slider and its current value display
    auto firefly_exposure_layout = new QHBoxLayout();
    firefly_exposure_slider_ = new QSlider(Qt::Horizontal, this);
    firefly_exposure_slider_->setRange(firefly_exposure_min_, firefly_exposure_max_);
    firefly_exposure_slider_->setMinimumHeight(80);
    firefly_exposure_slider_->setTickPosition(QSlider::TicksBelow);
    firefly_exposure_slider_->setTickInterval(500);
    firefly_exposure_slider_->setValue(firefly_exposure_);
    firefly_exposure_layout->addWidget(firefly_exposure_slider_);

    firefly_exposure_value_label_ = new QLabel(QString::number(firefly_exposure_slider_->value()), this);
    firefly_exposure_value_label_->setAlignment(Qt::AlignCenter);
    firefly_exposure_layout->addWidget(firefly_exposure_value_label_);
    middle_layout->addLayout(firefly_exposure_layout);

    // Update the value label when the slider moves
    connect(firefly_exposure_slider_, &QSlider::valueChanged, this,
            [this](int value) { firefly_exposure_value_label_->setText(QString::number(value)); });

    // Ximea Gain Control
    ximea_gain_label_ = new QLabel(
    QString("Ximea Gain (%1 - %2 dB)")
        .arg(ximea_gain_min_)
        .arg(ximea_gain_max_),
    this);
    ximea_gain_label_->setAlignment(Qt::AlignCenter);
    middle_layout->addWidget(ximea_gain_label_);

    auto ximea_gain_layout = new QHBoxLayout();
    ximea_gain_slider_ = new QSlider(Qt::Horizontal, this);
    ximea_gain_slider_->setRange(static_cast<int>(ximea_gain_min_ * 10), static_cast<int>(ximea_gain_max_ * 10));
    ximea_gain_slider_->setMinimumHeight(80);
    ximea_gain_slider_->setTickPosition(QSlider::TicksBelow);
    ximea_gain_slider_->setTickInterval(1);
    ximea_gain_slider_->setValue(static_cast<int>(ximea_gain_ * 10));
    ximea_gain_layout->addWidget(ximea_gain_slider_);

    ximea_gain_value_label_ = new QLabel(QString::number(ximea_gain_slider_->value() / 10.0, 'f', 1), this);
    ximea_gain_value_label_->setAlignment(Qt::AlignCenter);
    ximea_gain_layout->addWidget(ximea_gain_value_label_);
    middle_layout->addLayout(ximea_gain_layout);

    connect(ximea_gain_slider_, &QSlider::valueChanged, this,
            [this](int value) { ximea_gain_value_label_->setText(QString::number(value / 10.0, 'f', 1)); });

    // Ximea Exposure Control
    ximea_exposure_label_ = new QLabel(
    QString("Ximea Exposure (%1 - %2 us)")
        .arg(ximea_exposure_min_)
        .arg(ximea_exposure_max_),
    this);
    ximea_exposure_label_->setAlignment(Qt::AlignCenter);
    middle_layout->addWidget(ximea_exposure_label_);

    auto ximea_exposure_layout = new QHBoxLayout();
    ximea_exposure_slider_ = new QSlider(Qt::Horizontal, this);
    ximea_exposure_slider_->setRange(ximea_exposure_min_, ximea_exposure_max_);
    ximea_exposure_slider_->setMinimumHeight(80);
    ximea_exposure_slider_->setTickPosition(QSlider::TicksBelow);
    ximea_exposure_slider_->setTickInterval(500);
    ximea_exposure_slider_->setValue(ximea_exposure_);
    ximea_exposure_layout->addWidget(ximea_exposure_slider_);

    ximea_exposure_value_label_ = new QLabel(QString::number(ximea_exposure_slider_->value()), this);
    ximea_exposure_value_label_->setAlignment(Qt::AlignCenter);
    ximea_exposure_layout->addWidget(ximea_exposure_value_label_);
    middle_layout->addLayout(ximea_exposure_layout);

    connect(ximea_exposure_slider_, &QSlider::valueChanged, this,
            [this](int value) { ximea_exposure_value_label_->setText(QString::number(value)); });

    // Update Settings Button
    update_settings_button_ = new QPushButton("Update Settings", this);
    update_settings_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    update_settings_button_->setMinimumHeight(80);
    middle_layout->addWidget(update_settings_button_);
    connect(update_settings_button_, &QPushButton::clicked, this, &DirectorGui::handle_update_settings);

    // New Calibration Buttons
    ffc_calibrate_button_ = new QPushButton("FFC Calibrate", this);
    ffc_calibrate_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ffc_calibrate_button_->setMinimumHeight(80);
    middle_layout->addWidget(ffc_calibrate_button_);
    connect(ffc_calibrate_button_, &QPushButton::clicked, this, &DirectorGui::handle_ffc_calibrate_button_click);

    dark_calibrate_button_ = new QPushButton("Dark Calibrate", this);
    dark_calibrate_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    dark_calibrate_button_->setMinimumHeight(80);
    middle_layout->addWidget(dark_calibrate_button_);
    connect(dark_calibrate_button_, &QPushButton::clicked, this, &DirectorGui::handle_dark_calibrate_button_click);

    // Add middle layout with 1/4 stretch to the main layout
    main_layout->addLayout(middle_layout, 1); // Stretch factor = 1

    // --- Right Column: Image Displays ---
    auto right_layout = new QVBoxLayout();
    right_layout->setSpacing(10);

    // Add image labels to the right layout
    firefly_left_label_ = new QLabel(this);
    firefly_left_label_->setStyleSheet("border: 1px solid black;");
    firefly_left_label_->setScaledContents(true);
    firefly_left_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    right_layout->addWidget(firefly_left_label_);

    ximea_label_ = new QLabel(this);
    ximea_label_->setStyleSheet("border: 1px solid black;");
    ximea_label_->setScaledContents(true);
    ximea_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    right_layout->addWidget(ximea_label_);

    zed_left_label_ = new QLabel(this);
    zed_left_label_->setStyleSheet("border: 1px solid black;");
    zed_left_label_->setScaledContents(true);
    zed_left_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    right_layout->addWidget(zed_left_label_);

    // Add right layout with 2/4 stretch to the main layout
    main_layout->addLayout(right_layout, 2); // Stretch factor = 2

    // Set the main layout as the central layout
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
        QSlider::groove:horizontal {
          border: 1px solid #999999;
          height: 20px;
          background: #d3d3d3;
          margin: 0px;
          border-radius: 10px;
        }
        QSlider::handle:horizontal {
          background: #0078D7;
          border: 1px solid #5c5c5c;
          width: 80px;
          height: 80px;
          margin: -10px 0;
          border-radius: 20px;
        }
    )";
    setStyleSheet(style_sheet);
}

// -----------------------------------------------------------------------------
// Slot: Capture Button
// -----------------------------------------------------------------------------
void DirectorGui::handle_capture_button_click()
{
    auto message = std_msgs::msg::String();
    message.data = "Capture " + std::to_string(image_count_++);
    publisher_->publish(message);

    status_label_->setText(QString("Triggered: %1").arg(QString::fromStdString(message.data)));
    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
}

// -----------------------------------------------------------------------------
// Slot: Record Button (unchanged)
// -----------------------------------------------------------------------------
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
                    message.data = "Capture " + std::to_string(image_count_++);
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

// -----------------------------------------------------------------------------
// Slot: Rosbag Button (updated to record only required topics)
// -----------------------------------------------------------------------------
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
        std::string cmd = "ros2 bag record -o " + dir_name + " " + 
            director_topic_ + " " + 
            firefly_left_topic_ + " " + 
            firefly_right_topic_ + " " + 
            ximea_topic_ + " " + 
            zed_left_topic_ + " " + 
            zed_right_topic_ + " " + 
            zed_imu_topic_ + 
            " > /dev/null 2>&1 & echo $!";

        FILE *pipe = popen(cmd.c_str(), "r");
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

// -----------------------------------------------------------------------------
// Callback: Director Message
// -----------------------------------------------------------------------------
void DirectorGui::director_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Instead of directly appending to log_area_, emit the signal
    emit newDirectorMessage(QString::fromStdString(msg->data));
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

// -----------------------------------------------------------------------------
// Slot: Update Settings (Triggered only when the update button is pressed)
// -----------------------------------------------------------------------------
void DirectorGui::handle_update_settings()
{
    if (flash_duration_ != flash_duration_slider_->value())
    {
        flash_duration_ = flash_duration_slider_->value();
        RCLCPP_INFO(this->get_logger(), "Set flash duration to: %d", flash_duration_);

        auto message = std_msgs::msg::String();
        message.data = "Flash duration: " + std::to_string(flash_duration_);
        publisher_->publish(message);
    }

    if (firefly_exposure_ != firefly_exposure_slider_->value())
    {
        firefly_exposure_ = firefly_exposure_slider_->value();
        RCLCPP_INFO(this->get_logger(), "Set firefly exposure to: %d", firefly_exposure_);

        auto message = std_msgs::msg::String();
        message.data = "Firefly exposure: " + std::to_string(firefly_exposure_);
        publisher_->publish(message);
    }

    if (ximea_gain_ != ximea_gain_slider_->value())
    {
        ximea_gain_ = ximea_gain_slider_->value() / 10.0;
        RCLCPP_INFO(this->get_logger(), "Set ximea gain to: %.1f", ximea_gain_);

        auto message = std_msgs::msg::String();
        message.data = "Ximea gain: " + std::to_string(ximea_gain_);
        publisher_->publish(message);
    }

    if (ximea_exposure_ != ximea_exposure_slider_->value())
    {
        ximea_exposure_ = ximea_exposure_slider_->value();
        RCLCPP_INFO(this->get_logger(), "Set ximea exposure to: %d", ximea_exposure_);

        auto message = std_msgs::msg::String();
        message.data = "Ximea exposure: " + std::to_string(ximea_exposure_);
        publisher_->publish(message);
    }
}

// -----------------------------------------------------------------------------
// Stop Recording
// -----------------------------------------------------------------------------
void DirectorGui::stop_recording()
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
}

// -----------------------------------------------------------------------------
// Slot: FFC Calibrate Button (Sends a directive over the director topic)
// -----------------------------------------------------------------------------
void DirectorGui::handle_ffc_calibrate_button_click()
{
    stop_recording();
    std_msgs::msg::String message;
    message.data = "FFC calibrate";
    publisher_->publish(message);
    status_label_->setText("FFC Calibration initiated");
    RCLCPP_INFO(this->get_logger(), "Sent: %s", message.data.c_str());
}

// -----------------------------------------------------------------------------
// Slot: Dark Calibrate Button (Sends a directive over the director topic)
// -----------------------------------------------------------------------------
void DirectorGui::handle_dark_calibrate_button_click()
{
    stop_recording();
    std_msgs::msg::String message;
    message.data = "Dark calibrate";
    publisher_->publish(message);
    status_label_->setText("Dark Calibration initiated");
    RCLCPP_INFO(this->get_logger(), "Sent: %s", message.data.c_str());
}

// -----------------------------------------------------------------------------
// Callback: Image Processing
// -----------------------------------------------------------------------------
void DirectorGui::image_callback(const sensor_msgs::msg::Image::SharedPtr msg, QLabel *label)
{
    QtConcurrent::run([this, msg, label]()
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            QImage::Format image_format;

            // Determine the encoding and QImage format based on the number of channels
            if (msg->encoding == "mono8" || msg->encoding == "mono16" ||
                msg->encoding == "8UC1" || msg->encoding == "16UC1")
            {
                // Grayscale image
                cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
                image_format = QImage::Format_Grayscale8;
            }
            else if (msg->encoding == "bgr8" || msg->encoding == "bgra8" ||
                    msg->encoding == "rgb8" || msg->encoding == "rgba8")
            {
                // Color image
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                image_format = QImage::Format_BGR888;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                return;
            }

            // Determine aspect ratio
            double aspect_ratio = static_cast<double>(cv_ptr->image.cols) / cv_ptr->image.rows;

            // Calculate label size based on window dimensions
            int max_width = window_width_ / 3;
            int max_height = window_height_ / 3;
            int scaled_width = max_width;
            int scaled_height = static_cast<int>(scaled_width / aspect_ratio);

            // Calculate scaled size maintaining aspect ratio
            if (scaled_height > max_height)
            {
                scaled_height = max_height;
                scaled_width = static_cast<int>(scaled_height * aspect_ratio);
            }

            // Create QImage from cv::Mat
            QImage q_image_copy(cv_ptr->image.data,
                                cv_ptr->image.cols,
                                cv_ptr->image.rows,
                                static_cast<int>(cv_ptr->image.step),
                                image_format);

            // Ensure the QImage owns its data
            q_image_copy = q_image_copy.copy();

            QMetaObject::invokeMethod(this, [label, q_image_copy, scaled_width, scaled_height]()
            {
                label->setPixmap(QPixmap::fromImage(q_image_copy)
                    .scaled(scaled_width, scaled_height, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            });
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        } 
    });
}

// -----------------------------------------------------------------------------
// closeEvent
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
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
