from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Traffic Detection Node (DeepStream)
        Node(
            package='deepstream_tracker',
            executable='deepstream_tracker_node',
            name='traffic_detect_node',
            output='screen',
            parameters=[{
                'source_topic': '/image_raw'
            }]
        ),

        # 2. USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            # REMAPPING: This ensures the camera publishes to exactly where the tracker listens
            remappings=[
                ('image_raw', '/image_raw')
            ],
            parameters=[{
                "video_device": "/dev/video0",
                "image_width": 1280,
                "image_height": 720,
                "framerate": 30.0,
                "pixel_format": "uyvy", # Ensure your tracker handles UYVY, otherwise use "yuyv" or "mjpeg"
                "io_method": "mmap",
                "brightness": 2,
                "contrast": 3,
                "saturation": 4,
                "sharpness": 1,
                "gain": 2,
                "white_balance_automatic": True,
                "auto_exposure": 3
            }]
        ),
        
        # 3. Visualization Node
        Node(
            package='detections_img',
            executable='detections_img_node',
            name='detections_img_node',
            output='screen',
            # Assuming this node also needs to subscribe to the results, 
            # you might need parameters or remappings here too.
        )
    ])
