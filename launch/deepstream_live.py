from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Default HLS stream URL (Caltrans camera)
    default_hls_url = "https://wzmedia.dot.ca.gov/D5/1atFoothillBlvd.stream/playlist.m3u8"

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_hls_url,
        description='URL to the HLS video stream (.m3u8)'
    )

    # 1. MASTER TRACKER NODE (HLS Mode)
    # Reads the stream directly using souphttpsrc -> hlsdemux
    tracker_node = Node(
        package='deepstream_tracker',
        executable='deepstream_tracker_node',
        name='deepstream_tracker',
        output='screen',
        parameters=[{
            'source_type': 1,  # 1 = HLS / HTTP Stream
            'video_path': LaunchConfiguration('video_path')
        }]
    )

    # 2. VISUALIZER
    # Draws boxes on the images published by the tracker
    vis_node = Node(
        package='detections_img',
        executable='detections_img_node',
        name='detections_img',
        output='screen'
    )

    return LaunchDescription([
        video_path_arg,
        tracker_node,
        vis_node
    ])
