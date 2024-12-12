cmake_minimum_required(VERSION 3.5)
project(multi_cam_rig_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Find Ximea SDK
find_path(XIMEA_INCLUDE_DIR
    NAMES xiApi.h
    PATHS /opt/XIMEA/include
)
find_library(XIMEA_LIBRARY
    NAMES m3api
    PATHS /opt/XIMEA/lib
)
if(NOT XIMEA_INCLUDE_DIR)
    message(FATAL_ERROR "Ximea SDK not found. Please install it from https://www.ximea.com/support/wiki/apis/XIMEA_Linux_Software_Package")
endif()

# Find ZED SDK
find_path(ZED_INCLUDE_DIR
    NAMES sl/Camera.hpp
    PATHS /usr/local/zed/include
)
find_library(ZED_LIBRARY
    NAMES sl_zed
    PATHS /usr/local/zed/lib
)

if(NOT ZED_INCLUDE_DIR OR NOT ZED_LIBRARY)
    message(FATAL_ERROR "ZED SDK not found. Please install it from https://www.stereolabs.com/developers/release/")
endif()

# Include directories
include_directories(
    include
    ${XIMEA_INCLUDE_DIR}
    ${ZED_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)
link_directories(
    ${XIMEA_LIBRARY}
    ${ZED_LIBRARY}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARY_DIRS}
)

# Add MOC processing for the Qt header
set(CMAKE_AUTOMOC ON) # Automatically process headers with Q_OBJECT

# Create the executable
add_executable(director_gui
    src/director_gui.cpp
    include/multi_cam_rig_cpp/director_gui.hpp
)
target_link_libraries(director_gui Qt5::Core Qt5::Widgets)
ament_target_dependencies(director_gui rclcpp std_msgs sensor_msgs cv_bridge)

add_executable(firefly_capture_node src/firefly_capture_node.cpp)
ament_target_dependencies(firefly_capture_node rclcpp std_msgs sensor_msgs serial_driver)

add_executable(ximea_capture_node src/ximea_capture_node.cpp)
ament_target_dependencies(ximea_capture_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(ximea_capture_node ${XIMEA_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(zed_capture_node src/zed_capture_node.cpp)
ament_target_dependencies(zed_capture_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(zed_capture_node ${ZED_LIBRARY} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES})

# Install the executable
install(TARGETS
    director_gui
    firefly_capture_node
    ximea_capture_node
    zed_capture_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)


# Install the config files
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/config
)

# install the calibration files
install(DIRECTORY
    calibration
    DESTINATION share/${PROJECT_NAME}/calibration
)

ament_package()
