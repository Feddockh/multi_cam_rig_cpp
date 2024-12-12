# multi_cam_rig_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import datetime

def generate_launch_description():

    # Define the image topics
    director_topic = '/multi_cam_rig/director'
    firefly_left_image_topic = '/flir_node/firefly_left/image_raw'
    firefly_right_image_topic = '/flir_node/firefly_right/image_raw'
    ximea_image_topic = '/multi_cam_rig/ximea/image'
    zed_left_image_topic = '/multi_cam_rig/zed/left_image'
    zed_right_image_topic = '/multi_cam_rig/zed/right_image'

    # Set to True to save images to disk
    save_images = False
    save_dir = ''
    if save_images:

        # Create the data directory
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        print(f'Workspace directory: {workspace_dir}')
        data_dir = os.path.join(workspace_dir, 'multi_cam_rig_cpp', 'data')
        os.makedirs(data_dir, exist_ok=True)
        
        # Create the save directory with the current date
        save_dir_name = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        save_dir = os.path.join(data_dir, save_dir_name)
        print(f'Saving images to: {save_dir}')
        os.makedirs(save_dir, exist_ok=True)

    # Create the director gui node
    director_gui_node = Node(
        package='multi_cam_rig_cpp',
        executable='director_gui',
        name='director_gui',
        output='screen',
        parameters=[{
            'save_images': save_images,
            'save_dir': save_dir,
            'director_topic': director_topic,
            'firefly_left_image_topic': firefly_left_image_topic,
            'firefly_right_image_topic': firefly_right_image_topic,
            'ximea_image_topic': ximea_image_topic,
            'zed_left_image_topic': zed_left_image_topic,
            'zed_right_image_topic': zed_right_image_topic,
        }]
    )

    # Create the firefly capture node
    firefly_capture_node = Node(
        package='multi_cam_rig_cpp',
        executable='firefly_capture_node',
        name='firefly_capture_node',
        output='screen',
        parameters=[{
            'save_images': save_images,
            'save_dir': save_dir,
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
            'save_images': save_images,
            'save_dir': save_dir,
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
            'save_images': save_images,
            'save_dir': save_dir,
            'director_topic': director_topic,
            'left_image_topic': zed_left_image_topic,
            'right_image_topic': zed_right_image_topic,
        }]
    )

    return LaunchDescription([
        director_gui_node,
        firefly_capture_node,
        ximea_capture_node,
        zed_capture_node
    ])
