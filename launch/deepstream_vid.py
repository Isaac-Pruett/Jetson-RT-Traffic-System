from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. MASTER TRACKER NODE
    # Reads video, tracks objects, and publishes BOTH /traffic_detections and /image_raw
    # Includes the "Leaky Queue" fix to prevent freezing.
    tracker_node = Node(
        package='deepstream_tracker',
        executable='deepstream_tracker_node',
        name='deepstream_tracker',
        output='screen',
        parameters=[{
            'source_type': 0,  # 0 = File, 1 = Stream, 2 = Camera
            #'video_path': '/home/nvidia/Jetson-RT-Traffic-System/video/fixed_vid.mp4'
            'video_path': '/home/nvidia/Jetson-RT-Traffic-System/video/caltrans_captured.mp4'
        }]
    )

    # 2. VISUALIZER
    # Subscribes to /traffic_detections (Reliable) and /image_raw (Best Effort)
    # Draws the bounding boxes on the image.
    vis_node = Node(
        package='detections_img',
        executable='detections_img_node',
        name='detections_img',
        output='screen'
    )
    


    return LaunchDescription([
        tracker_node,
        vis_node
    ])
