from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- User-friendly knobs
    start_cameras = LaunchConfiguration('start_cameras')
    manage_previews = LaunchConfiguration('manage_previews')
    respawn_cameras = LaunchConfiguration('respawn_cameras')

    # Convert LaunchConfiguration "true/false" strings to real bool params
    start_cameras_bool = ParameterValue(start_cameras, value_type=bool)
    manage_previews_bool = ParameterValue(manage_previews, value_type=bool)

    # Low-latency preview settings
    preview_w = 960
    preview_h = 540
    preview_fps = 20
    frame_us = int(1_000_000 / preview_fps)

    # Only start standalone camera_ros nodes if manage_previews:=false.
    cam_condition_no_respawn = IfCondition(
        PythonExpression([
            "'", start_cameras, "' == 'true' and '",
            respawn_cameras, "' == 'false' and '",
            manage_previews, "' == 'false'"
        ])
    )
    cam_condition_respawn = IfCondition(
        PythonExpression([
            "'", start_cameras, "' == 'true' and '",
            respawn_cameras, "' == 'true' and '",
            manage_previews, "' == 'false'"
        ])
    )

    # --- Camera previews (non-respawning)
    cam0 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera0',
        namespace='cam0',
        output='screen',
        parameters=[{
            'camera': 0,
            'role': 'viewfinder',
            'width': preview_w,
            'height': preview_h,
            'FrameDurationLimits': [frame_us, frame_us],
            'use_node_time': False,
        }],
        respawn=False,
        condition=cam_condition_no_respawn,
    )

    cam1 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera1',
        namespace='cam1',
        output='screen',
        parameters=[{
            'camera': 1,
            'role': 'viewfinder',
            'width': preview_w,
            'height': preview_h,
            'FrameDurationLimits': [frame_us, frame_us],
            'use_node_time': False,
        }],
        respawn=False,
        condition=cam_condition_no_respawn,
    )

    # --- Camera previews (respawning)
    cam0_r = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera0',
        namespace='cam0',
        output='screen',
        parameters=[{
            'camera': 0,
            'role': 'viewfinder',
            'width': preview_w,
            'height': preview_h,
            'FrameDurationLimits': [frame_us, frame_us],
            'use_node_time': False,
        }],
        respawn=True,
        respawn_delay=6.0,
        condition=cam_condition_respawn,
    )

    cam1_r = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera1',
        namespace='cam1',
        output='screen',
        parameters=[{
            'camera': 1,
            'role': 'viewfinder',
            'width': preview_w,
            'height': preview_h,
            'FrameDurationLimits': [frame_us, frame_us],
            'use_node_time': False,
        }],
        respawn=True,
        respawn_delay=6.0,
        condition=cam_condition_respawn,
    )

    # --- Capture service (12MP stills)
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

            # Preview pause/resume implementation
            'manage_previews': manage_previews_bool,
            'start_previews': start_cameras_bool,
            'pause_previews': True,

            'preview_width': preview_w,
            'preview_height': preview_h,
            'preview_fps': preview_fps,
            'preview_role': 'viewfinder',

            # Make preview topics match the UI defaults:
            # /cam0/camera/image_raw and /cam1/camera/image_raw
            'cam0_namespace': '/cam0',
            'cam1_namespace': '/cam1',
            'cam0_node_name': 'camera',
            'cam1_node_name': 'camera',
        }]
    )

    # --- Touch UI
    ui = Node(
        package='subsea_ui',
        executable='ui',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_cameras', default_value='true'),
        DeclareLaunchArgument('respawn_cameras', default_value='false'),
        DeclareLaunchArgument('manage_previews', default_value='true'),

        cam0,
        cam1,
        cam0_r,
        cam1_r,
        capture,
        ui,
    ])
