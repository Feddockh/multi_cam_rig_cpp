import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters for the calibration
    chessboard_size = "11x9"  # Number of internal corners in the checkerboard
    square_size = "0.0425"    # Size of a square in meters

    return LaunchDescription([
        # Calibration node
        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            name="stereo_calibration",
            arguments=[
                "--size", chessboard_size,
                "--square", square_size,
                "--approximate", "0.1"  # Allow 100ms slop for approximate synchronization
            ],
            remappings=[
                ("left", "/multi_cam_rig/zed/left_image"),  # Remap left image topic
                ("right", "/multi_cam_rig/ximea/image")  # Remap right image topic
            ],
            output="screen",
        ),
    ])
