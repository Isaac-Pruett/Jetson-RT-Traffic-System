from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Default V4L2 device path
    default_usb_device = "/dev/video0"

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_usb_device,
        description='Path to the USB camera device (e.g., /dev/video0)'
    )

    # --- THE HARDWARE TUNE ---
    # This runs the exact millisecond you launch the file, configuring the See3CAM
    set_camera_hw = ExecuteProcess(
        cmd=["""
            v4l2-ctl -d /dev/video0 \
            -c brightness=2 \
            -c contrast=3 \
            -c saturation=4 \
            -c sharpness=1 \
            -c gain=2 \
            -c white_balance_automatic=1 \
            -c auto_exposure=0
        """],
        shell=True
    )

    # 1. MASTER TRACKER NODE (USB Mode)
    tracker_node = Node(
        package='deepstream_tracker',
        executable='deepstream_tracker_node',
        name='deepstream_tracker',
        output='screen',
        parameters=[{
            'source_type': 2,  # 2 = USB / V4L2 Camera
            'video_path': LaunchConfiguration('video_path')
        }]
    )

    # 2. VISUALIZER
    vis_node = Node(
        package='detections_img',
        executable='detections_img_node',
        name='detections_img',
        output='screen'
    )

    return LaunchDescription([
        video_path_arg,
        set_camera_hw,      
        tracker_node,
        vis_node
    ])
