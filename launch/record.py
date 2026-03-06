from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                "video_device": "/dev/video0",
                "image_width": 1280,
                "image_height": 720,
                "framerate": 30.0,
                "pixel_format": "uyvy",
                "io_method": "mmap",

                "brightness": 3,
                "contrast": 3,
                "saturation": 3,
                "sharpness": 5,
                "gain": 5,
                "white_balance_automatic": True,
                "auto_exposure": 1
            }]
        ), 
    ])
