# multi_cam_rig_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Director GUI Node
        Node(
            package='multi_cam_rig_cpp',
            executable='director_gui',
            name='director_gui',
            output='screen',
            parameters=[{
                'director_topic': '/multi_cam_rig/director',
                'firefly_left_image_topic': '/flir_node/firefly_left/image_raw',
                'firefly_right_image_topic': '/flir_node/firefly_right/image_raw',
                'ximea_image_topic': '/multi_cam_rig/ximea/image',
                'zed_left_image_topic': '/multi_cam_rig/zed/left_image',
                'zed_right_image_topic': '/multi_cam_rig/zed/right_image',
            }]
        ),

        # Firefly Capture Node
        Node(
            package='multi_cam_rig_cpp',
            executable='firefly_capture_node',
            name='firefly_capture_node',
            output='screen',
            parameters=[{
                'director_topic': '/multi_cam_rig/director',
                'left_image_topic': '/flir_node/firefly_left/image_raw',
                'right_image_topic': '/flir_node/firefly_right/image_raw',
            }]
        ),
        
        # Ximea Capture Node
        Node(
            package='multi_cam_rig_cpp',
            executable='ximea_capture_node',
            name='ximea_capture_node',
            output='screen',
            parameters=[{
                'director_topic': '/multi_cam_rig/director',
                'ximea_image_topic': '/multi_cam_rig/ximea/image',
            }]
        ),
        
        # Zed Capture Node
        Node(
            package='multi_cam_rig_cpp',
            executable='zed_capture_node',
            name='zed_capture_node',
            output='screen',
            parameters=[{
                'director_topic': '/multi_cam_rig/director',
                'left_image_topic': '/multi_cam_rig/zed/left_image',
                'right_image_topic': '/multi_cam_rig/zed/right_image',
            }]
        ),
    ])
