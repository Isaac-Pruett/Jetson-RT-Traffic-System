from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        #default_value= os.path.expanduser('~/Jetson-RT-Traffic-System/video/testvid2.mp4'),
        default_value= os.path.expanduser('~/Jetson-RT-Traffic-System/video/highwaytest2.mp4'),
        #default_value= os.path.expanduser('~/Jetson-RT-Traffic-System/video/Car_ex1.mp4'),
        
        description='Path to the video file to publish'
    )

    return LaunchDescription([
        video_path_arg,

        Node(
            package='deepstream_tracker',
            executable='deepstream_tracker_node',
            name='traffic_detect_node',
            output='screen',
            parameters=[{
                #'fps': 8.0,
                'source_topic': '/image_raw'

            }]
        ),
        Node(
            package='video_streamer_cpp',
            executable='video_streamer_node',
            name='video_publisher_node',
            output='screen',
            parameters=[{
                'video_filepath': LaunchConfiguration('video_path'),
                'width': 960,
                'height': 544,
                'fps': 60
                }]
        ),

        
        Node(
            package='detections_img',
            executable='detections_img_node',
            name='detections_img_node',
            output='screen'
        ),
    ])
