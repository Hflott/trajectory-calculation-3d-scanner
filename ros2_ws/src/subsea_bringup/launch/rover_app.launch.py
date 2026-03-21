from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    EmitEvent,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable, TextSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- User-friendly knobs
    start_cameras = LaunchConfiguration('start_cameras')
    manage_previews = LaunchConfiguration('manage_previews')
    respawn_cameras = LaunchConfiguration('respawn_cameras')
    start_localization = LaunchConfiguration('start_localization')
    capture_mode = LaunchConfiguration('capture_mode')

    # Convert LaunchConfiguration "true/false" strings to bool params
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

    # --- Ensure the *correct* libcamera + IPA modules are used (prevents "waiting..." + serializer crashes)
    camera_ws = os.path.expanduser("~/camera_ws/install")
    libcamera_lib = os.path.join(camera_ws, "libcamera", "lib")
    ipa_dir = os.path.join(camera_ws, "libcamera", "lib", "libcamera", "ipa")

    env_actions = []
    if os.path.isdir(libcamera_lib):
        env_actions.append(
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    TextSubstitution(text=libcamera_lib + ":"),
                    EnvironmentVariable("LD_LIBRARY_PATH"),
                ],
            )
        )
    if os.path.isdir(ipa_dir):
        env_actions.append(SetEnvironmentVariable(name="LIBCAMERA_IPA_MODULE_PATH", value=ipa_dir))

    # --- Camera previews (ONLY used when manage_previews:=false)
    cam0 = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
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
        name='camera',
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

    cam0_r = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
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
        name='camera',
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

    # --- Capture service (stream-synced by default; still mode optional)
    capture = Node(
        package='subsea_capture',
        executable='capture_service',
        output='screen',
        parameters=[{
            'cam0_index': 0,
            'cam1_index': 1,
            'width': 4056,
            'height': 3040,
            'timeout_ms': 6000,     # give still capture some breathing room
            'warmup_ms': 700,
            'default_quality': 95,

            # Preview pause/resume
            'manage_previews': manage_previews_bool,
            'start_previews': start_cameras_bool,
            'pause_previews': True,
            'fallback_black_previews': False,

            # Timestamp-accurate capture from live image stream (recommended for
            # GNSS/IMU motion compensation workflows).
            'capture_mode': capture_mode,
            'stream_wait_s': 1.0,
            'stream_max_frame_age_s': 1.0,
            'write_capture_metadata': True,
            'sensor_buffer_s': 20.0,
            'gnss_fix_topic': '/fix',
            'gnss_time_ref_topic': '/time_reference',
            'gnss_imu_topic': '/imu/data',

            'preview_width': preview_w,
            'preview_height': preview_h,
            'preview_fps': preview_fps,
            'preview_role': 'viewfinder',

            # Make preview topics match the UI defaults:
            'cam0_namespace': '/cam0',
            'cam1_namespace': '/cam1',
            'cam0_node_name': 'camera',
            'cam1_node_name': 'camera',
            'use_local_libcamera_env': False,
        }]
    )

    # --- Touch UI
    ui = Node(
        package='subsea_ui',
        executable='ui',
        output='screen'
    )

    # When UI exits, shut down the whole launch (fixes "terminal never exits")
    shutdown_on_ui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=ui,
            on_exit=[EmitEvent(event=Shutdown(reason="UI closed"))],
        )
    )

    localization_launch = os.path.join(
        get_package_share_directory("subsea_localization"),
        "launch",
        "localization.launch.py",
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        condition=IfCondition(start_localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_cameras', default_value='true'),
        DeclareLaunchArgument('respawn_cameras', default_value='false'),
        DeclareLaunchArgument('manage_previews', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='false'),
        DeclareLaunchArgument('capture_mode', default_value='stream'),

        *env_actions,

        cam0, cam1, cam0_r, cam1_r,
        capture,
        ui,
        localization,
        shutdown_on_ui_exit,
    ])
