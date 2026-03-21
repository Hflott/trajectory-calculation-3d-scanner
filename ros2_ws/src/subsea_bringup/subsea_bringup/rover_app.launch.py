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
    enable_gpio_button = LaunchConfiguration('enable_gpio_button')
    gpio_button_pin = LaunchConfiguration('gpio_button_pin')
    gpio_button_debounce_ms = LaunchConfiguration('gpio_button_debounce_ms')
    preview_width = LaunchConfiguration('preview_width')
    preview_height = LaunchConfiguration('preview_height')
    preview_fps = LaunchConfiguration('preview_fps')
    preview_format = LaunchConfiguration('preview_format')
    ui_fps = LaunchConfiguration('ui_fps')
    odom_local_topic = LaunchConfiguration('odom_local_topic')
    odom_global_topic = LaunchConfiguration('odom_global_topic')
    capture_debug_topic = LaunchConfiguration('capture_debug_topic')

    # Convert LaunchConfiguration "true/false" strings to bool params
    start_cameras_bool = ParameterValue(start_cameras, value_type=bool)
    manage_previews_bool = ParameterValue(manage_previews, value_type=bool)
    enable_gpio_button_bool = ParameterValue(enable_gpio_button, value_type=bool)
    gpio_button_pin_int = ParameterValue(gpio_button_pin, value_type=int)
    gpio_button_debounce_ms_int = ParameterValue(gpio_button_debounce_ms, value_type=int)
    preview_w_int = ParameterValue(preview_width, value_type=int)
    preview_h_int = ParameterValue(preview_height, value_type=int)
    preview_fps_int = ParameterValue(preview_fps, value_type=int)
    ui_fps_int = ParameterValue(ui_fps, value_type=int)

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
            'width': preview_w_int,
            'height': preview_h_int,
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
            'width': preview_w_int,
            'height': preview_h_int,
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
            'width': preview_w_int,
            'height': preview_h_int,
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
            'width': preview_w_int,
            'height': preview_h_int,
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
            'default_quality': 100,

            # Preview pause/resume
            'manage_previews': manage_previews_bool,
            'start_previews': start_cameras_bool,
            'pause_previews': True,
            'fallback_black_previews': False,

            # Timestamp-accurate capture from live image stream (recommended for
            # GNSS/IMU motion compensation workflows).
            'capture_mode': capture_mode,
            'stream_wait_s': 1.0,
            'stream_initial_wait_s': 2.5,
            'stream_max_frame_age_s': 1.0,
            'stream_buffer_len': 60,
            'stream_pair_max_delta_ms': 80.0,
            'write_capture_metadata': True,
            'sensor_buffer_s': 20.0,
            'capture_event_topic': '/capture/events',
            'capture_debug_topic': capture_debug_topic,
            'gnss_fix_topic': '/fix',
            'gnss_time_ref_topic': '/time_reference',
            'gnss_imu_topic': '/imu/data',
            'odom_local_topic': odom_local_topic,
            'odom_global_topic': odom_global_topic,

            'preview_width': preview_w_int,
            'preview_height': preview_h_int,
            'preview_fps': preview_fps_int,
            'preview_format': preview_format,
            'preview_role': 'viewfinder',
            'preview_start_stagger_s': 0.2,
            'preview_restart_attempts': 3,
            'preview_restart_delay_s': 0.2,
            'preview_shutdown_timeout_s': 1.0,
            'device_release_timeout_s': 1.0,
            'capture_parallel': True,

            # Make preview topics match the UI defaults:
            'cam0_namespace': '/cam0',
            'cam1_namespace': '/cam1',
            'cam0_node_name': 'camera',
            'cam1_node_name': 'camera',
            'use_local_libcamera_env': False,
            'sanitize_preview_env': True,
            'gpio_trigger_enable': enable_gpio_button_bool,
            'gpio_trigger_chip': '/dev/gpiochip4',
            'gpio_trigger_line': gpio_button_pin_int,
            'gpio_trigger_active_low': True,
            'gpio_trigger_debounce_ms': gpio_button_debounce_ms_int,
            'gpio_trigger_cooldown_ms': 700,
            'gpio_trigger_session_prefix': 'btn',
        }]
    )

    # --- Touch UI
    ui = Node(
        package='subsea_ui',
        executable='ui',
        output='screen',
        parameters=[{
            'ui_fps': ui_fps_int,
            'preview_fps': preview_fps_int,
            'capture_node': '/capture_service',
            'capture_event_topic': '/capture/events',
            'capture_debug_topic': capture_debug_topic,
        }],
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
        DeclareLaunchArgument('enable_gpio_button', default_value='true'),
        DeclareLaunchArgument('gpio_button_pin', default_value='24'),
        DeclareLaunchArgument('gpio_button_debounce_ms', default_value='40'),
        DeclareLaunchArgument('preview_width', default_value='960'),
        DeclareLaunchArgument('preview_height', default_value='540'),
        DeclareLaunchArgument('preview_fps', default_value='15'),
        DeclareLaunchArgument('preview_format', default_value='RGB888'),
        DeclareLaunchArgument('ui_fps', default_value='12'),
        DeclareLaunchArgument('odom_local_topic', default_value='/odometry/local'),
        DeclareLaunchArgument('odom_global_topic', default_value='/odometry/global'),
        DeclareLaunchArgument('capture_debug_topic', default_value='/capture/debug'),

        *env_actions,

        cam0, cam1, cam0_r, cam1_r,
        capture,
        ui,
        localization,
        shutdown_on_ui_exit,
    ])
