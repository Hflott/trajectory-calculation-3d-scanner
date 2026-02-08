from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Low-latency preview settings (change if you want)
    preview_w = 960
    preview_h = 540
    preview_fps = 20

    cam0 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera0',
        namespace='cam0',
        output='screen',
        parameters=[{
            'camera': 0,
            'width': preview_w,
            'height': preview_h,
            'fps': preview_fps,
        }],
        respawn=True,
        respawn_delay=6.0,
    )

    cam1 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera1',
        namespace='cam1',
        output='screen',
        parameters=[{
            'camera': 1,
            'width': preview_w,
            'height': preview_h,
            'fps': preview_fps,
        }],
        respawn=True,
        respawn_delay=6.0,
    )

    capture = Node(
        package='subsea_capture',
        executable='capture_service',
        output='screen',
        parameters=[{
            'cam0_index': 0,
            'cam1_index': 1,
            'width': 4056,
            'height': 3040,
            'timeout_ms': 250,
            'warmup_ms': 250,
            'default_quality': 95,
        }]
    )

    ui = Node(
        package='subsea_ui',
        executable='ui',
        output='screen'
    )

    return LaunchDescription([cam0, cam1, capture, ui])
