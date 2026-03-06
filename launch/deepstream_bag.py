from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([
        

            Node(
            package='deepstream_tracker',
            executable='deepstream_tracker_node',
            name='traffic_detect_node',
            output='screen',
            parameters=[{
                'source_topic': '/image_raw'

            }]
        ),

        Node(
            package='detections_img',
            executable='detections_img_node',
            name='detections_img_node',
            output='screen'
        )
    ])
