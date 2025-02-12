# multi_cam_rig_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Define the image topics
    director_topic = '/multi_cam_rig/director'
    firefly_left_image_topic = '/flir_node/firefly_left/image_raw'
    firefly_right_image_topic = '/flir_node/firefly_right/image_raw'
    ximea_image_topic = '/multi_cam_rig/ximea/image'
    zed_left_image_topic = '/multi_cam_rig/zed/left_image'
    zed_right_image_topic = '/multi_cam_rig/zed/right_image'
    zed_imu_topic = '/multi_cam_rig/zed/imu'

    # Save options
    data_dir = '~/data/rivendale_2-12-2025'

    # Create the director gui node
    director_gui_node = Node(
        package='multi_cam_rig_cpp',
        executable='director_gui',
        name='director_gui',
        output='screen',
        parameters=[{
            'data_dir': data_dir,
            'director_topic': director_topic,
            'firefly_left_image_topic': firefly_left_image_topic,
            'firefly_right_image_topic': firefly_right_image_topic,
            'ximea_image_topic': ximea_image_topic,
            'zed_left_image_topic': zed_left_image_topic,
            'zed_right_image_topic': zed_right_image_topic,
            'zed_imu_topic': zed_imu_topic,
        }]
    )

    # Include the firefly_synchronized launch file
    firefly_synchronized_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('multi_cam_rig_cpp'), 'launch', 'firefly_synchronized.launch.py'))
    )

    # Create the firefly capture node
    firefly_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='firefly_capture_node',
        name='firefly_capture_node',
        output='screen',
        parameters=[{
            'director_topic': director_topic,
            'left_image_topic': firefly_left_image_topic,
            'right_image_topic': firefly_right_image_topic,
        }]
    )

    # Create the ximea capture node
    ximea_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='ximea_capture_node',
        name='ximea_capture_node',
        output='screen',
        parameters=[{
            'data_dir': data_dir,
            'director_topic': director_topic,
            'ximea_image_topic': ximea_image_topic,
        }]
    )

    # Create the zed capture node
    zed_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='zed_capture_node',
        name='zed_capture_node',
        output='screen',
        parameters=[{
            'director_topic': director_topic,
            'left_image_topic': zed_left_image_topic,
            'right_image_topic': zed_right_image_topic,
            'imu_topic': zed_imu_topic
        }]
    )

    # Event handler to shutdown everything when director_gui_node exits
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=director_gui_node,
            on_exit=[Shutdown()]  # Shut down all actions in this launch description
        )
    )

    launch_description = LaunchDescription([
        director_gui_node,
        firefly_synchronized_launch,
        firefly_capture_node,
        ximea_capture_node,
        zed_capture_node,
        exit_handler
    ])

    return launch_description
