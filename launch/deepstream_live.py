from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # HLS stream URL (Caltrans camera)
    hls_url = "https://wzmedia.dot.ca.gov/D5/1atFoothillBlvd.stream/playlist.m3u8"

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=hls_url,
        description='Path or URL to the video stream (supports HLS .m3u8 URLs)'
    )

    return LaunchDescription([
        video_path_arg,

        # DeepStream tracking/detection node
        Node(
            package='deepstream_tracker',
            executable='deepstream_tracker_node',
            name='traffic_detect_node',
            output='screen',
            parameters=[{
                'source_topic': '/image_raw'
            }]
        ),

        # Video streamer node (publishes frames from .m3u8 source)
        Node(
            package='video_streamer_cpp',
            executable='video_streamer_node',
            name='video_publisher_node',
            output='screen',
            parameters=[{
                'video_filepath': LaunchConfiguration('video_path'),
                'width': 960,
                'height': 544,
                'fps': 24  # 30 FPS is more realistic for HLS sources
            }]
        ),

        # Visualization or downstream processing node
        Node(
            package='detections_img',
            executable='detections_img_node',
            name='detections_img_node',
            output='screen'
        ),
    ])

