#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    # Declare launch arguments to choose which cameras to launch
    camera1_arg = DeclareLaunchArgument(
        'camera1', default_value='zed_left',
        description='Camera 1 to use (e.g. zed_left, ximea, firefly_left)'
    )
    camera2_arg = DeclareLaunchArgument(
        'camera2', default_value='zed_right',
        description='Camera 2 to use (e.g. zed_left, zed_right, firefly_left, firefly_right)'
    )

    # Get launch configuration variables
    camera1 = LaunchConfiguration('camera1')
    camera2 = LaunchConfiguration('camera2')

    # Define image topic names for each camera
    director_topic = '/multi_cam_rig/director'
    firefly_left_image_topic = '/flir_node/firefly_left/image_raw'
    firefly_right_image_topic = '/flir_node/firefly_right/image_raw'
    ximea_image_topic = '/multi_cam_rig/ximea/image'
    zed_left_image_topic = '/multi_cam_rig/zed/left_image'
    zed_right_image_topic = '/multi_cam_rig/zed/right_image'
    data_dir = '~/data/calibration'
    trigger_frequency = 2  # Hz

    # Conditions (unchanged)
    use_firefly = IfCondition(PythonExpression([
        '"', camera1, '" == "firefly_left" or "', camera1, '" == "firefly_right" or "',
        camera2, '" == "firefly_left" or "', camera2, '" == "firefly_right"'
    ]))
    use_ximea = IfCondition(PythonExpression([
        '"', camera1, '" == "ximea" or "', camera2, '" == "ximea"'
    ]))
    use_zed = IfCondition(PythonExpression([
        '"', camera1, '" == "zed_left" or "', camera1, '" == "zed_right" or "',
        camera2, '" == "zed_left" or "', camera2, '" == "zed_right"'
    ]))

    # Use valid Python ternary expressions by ensuring all literals are quoted.
    camera1_image_topic = PythonExpression([
        "'/flir_node/firefly_left/image_raw' if '", camera1, "' == 'firefly_left' else "
        "'/flir_node/firefly_right/image_raw' if '", camera1, "' == 'firefly_right' else "
        "'/multi_cam_rig/zed/left_image' if '", camera1, "' == 'zed_left' else "
        "'/multi_cam_rig/zed/right_image' if '", camera1, "' == 'zed_right' else "
        "'/multi_cam_rig/ximea/image' if '", camera1, "' == 'ximea' else 'UNKNOWN_CAMERA'"
    ])

    camera2_image_topic = PythonExpression([
        "'/flir_node/firefly_left/image_raw' if '", camera2, "' == 'firefly_left' else "
        "'/flir_node/firefly_right/image_raw' if '", camera2, "' == 'firefly_right' else "
        "'/multi_cam_rig/zed/left_image' if '", camera2, "' == 'zed_left' else "
        "'/multi_cam_rig/zed/right_image' if '", camera2, "' == 'zed_right' else "
        "'/multi_cam_rig/ximea/image' if '", camera2, "' == 'ximea' else 'UNKNOWN_CAMERA'"
    ])

    # List of nodes to launch
    nodes = []

    # --- Firefly Capture Node ---
    firefly_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='firefly_capture_node',
        name='firefly_capture_node',
        output='screen',
        parameters=[{
            'left_image_topic': firefly_left_image_topic,
            'right_image_topic': firefly_right_image_topic,
        }],
        condition=use_firefly
    )
    nodes.append(firefly_capture_node)

    # --- Ximea Capture Node ---
    ximea_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='ximea_capture_node',
        name='ximea_capture_node',
        output='screen',
        parameters=[{
            'data_dir': data_dir,
            'ximea_image_topic': ximea_image_topic,
        }],
        condition=use_ximea
    )
    nodes.append(ximea_capture_node)

    # --- Zed Capture Node ---
    zed_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='zed_capture_node',
        name='zed_capture_node',
        output='screen',
        parameters=[{
            'left_image_topic': zed_left_image_topic,
            'right_image_topic': zed_right_image_topic,
        }],
        condition=use_zed
    )
    nodes.append(zed_capture_node)

    # --- Camera Calibration Node ---
    calibration_node = Node(
        package='multi_cam_rig_cpp',
        executable='calibration_node.py',  # or 'calibration_node' if installed without .py
        name='camera_calibration_node',
        output='screen',
        parameters=[{
            'director_topic': director_topic,
            'left_image_topic': camera1_image_topic,
            'right_image_topic': camera2_image_topic,
            'trigger_frequency': trigger_frequency
        }]
    )
    nodes.append(calibration_node)

    return LaunchDescription([camera1_arg, camera2_arg] + nodes)
