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
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- User-friendly knobs
    start_gpsd_client = LaunchConfiguration('start_gpsd_client')
    use_gpsd_json_bridge = LaunchConfiguration('use_gpsd_json_bridge')
    gpsd_host = LaunchConfiguration('gpsd_host')
    gpsd_port = LaunchConfiguration('gpsd_port')
    start_imu_node = LaunchConfiguration('start_imu_node')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    imu_rate_hz = LaunchConfiguration('imu_rate_hz')
    imu_i2c_address = LaunchConfiguration('imu_i2c_address')
    imu_i2c_bus = LaunchConfiguration('imu_i2c_bus')
    imu_timestamp_mode = LaunchConfiguration('imu_timestamp_mode')
    imu_enable_rotation = LaunchConfiguration('imu_enable_rotation')
    imu_enable_accel = LaunchConfiguration('imu_enable_accel')
    imu_enable_gyro = LaunchConfiguration('imu_enable_gyro')
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
    preview_ui_width = LaunchConfiguration('preview_ui_width')
    preview_ui_height = LaunchConfiguration('preview_ui_height')
    preview_ui_fps = LaunchConfiguration('preview_ui_fps')
    preview_format = LaunchConfiguration('preview_format')
    swap_preview_feeds = LaunchConfiguration('swap_preview_feeds')
    cam0_orientation = LaunchConfiguration('cam0_orientation')
    cam1_orientation = LaunchConfiguration('cam1_orientation')
    preview_source_applies_orientation = LaunchConfiguration('preview_source_applies_orientation')
    ui_fps = LaunchConfiguration('ui_fps')
    odom_local_topic = LaunchConfiguration('odom_local_topic')
    odom_global_topic = LaunchConfiguration('odom_global_topic')
    capture_debug_topic = LaunchConfiguration('capture_debug_topic')
    trajectory_sample_rate_hz = LaunchConfiguration('trajectory_sample_rate_hz')
    trajectory_window_ms = LaunchConfiguration('trajectory_window_ms')
    write_trajectory_csv = LaunchConfiguration('write_trajectory_csv')
    enable_motion_deblur = LaunchConfiguration('enable_motion_deblur')
    deblur_exposure_ms = LaunchConfiguration('deblur_exposure_ms')
    deblur_exposure_time_us = LaunchConfiguration('deblur_exposure_time_us')
    deblur_image_stamp_reference = LaunchConfiguration('deblur_image_stamp_reference')
    deblur_timestamp_source = LaunchConfiguration('deblur_timestamp_source')
    deblur_fov_deg = LaunchConfiguration('deblur_fov_deg')
    deblur_strength = LaunchConfiguration('deblur_strength')
    deblur_min_kernel_px = LaunchConfiguration('deblur_min_kernel_px')
    deblur_max_kernel_px = LaunchConfiguration('deblur_max_kernel_px')
    deblur_iterations = LaunchConfiguration('deblur_iterations')
    deblur_method = LaunchConfiguration('deblur_method')
    deblur_wiener_snr = LaunchConfiguration('deblur_wiener_snr')
    deblur_imu_to_cam_yaw_deg = LaunchConfiguration('deblur_imu_to_cam_yaw_deg')
    deblur_use_translation = LaunchConfiguration('deblur_use_translation')
    deblur_assumed_depth_m = LaunchConfiguration('deblur_assumed_depth_m')
    deblur_max_odom_age_ms = LaunchConfiguration('deblur_max_odom_age_ms')
    deblur_allow_nearest_fallback = LaunchConfiguration('deblur_allow_nearest_fallback')
    deblur_max_integration_gap_ms = LaunchConfiguration('deblur_max_integration_gap_ms')
    deblur_require_time_reference = LaunchConfiguration('deblur_require_time_reference')
    deblur_max_time_reference_age_ms = LaunchConfiguration('deblur_max_time_reference_age_ms')

    # Convert LaunchConfiguration "true/false" strings to bool params
    start_cameras_bool = ParameterValue(start_cameras, value_type=bool)
    manage_previews_bool = ParameterValue(manage_previews, value_type=bool)
    enable_gpio_button_bool = ParameterValue(enable_gpio_button, value_type=bool)
    gpio_button_pin_int = ParameterValue(gpio_button_pin, value_type=int)
    gpio_button_debounce_ms_int = ParameterValue(gpio_button_debounce_ms, value_type=int)
    gpsd_port_int = ParameterValue(gpsd_port, value_type=int)
    imu_rate_hz_float = ParameterValue(imu_rate_hz, value_type=float)
    imu_i2c_address_int = ParameterValue(imu_i2c_address, value_type=int)
    imu_i2c_bus_int = ParameterValue(imu_i2c_bus, value_type=int)
    imu_enable_rotation_bool = ParameterValue(imu_enable_rotation, value_type=bool)
    imu_enable_accel_bool = ParameterValue(imu_enable_accel, value_type=bool)
    imu_enable_gyro_bool = ParameterValue(imu_enable_gyro, value_type=bool)
    preview_w_int = ParameterValue(preview_width, value_type=int)
    preview_h_int = ParameterValue(preview_height, value_type=int)
    preview_fps_int = ParameterValue(preview_fps, value_type=int)
    preview_ui_w_int = ParameterValue(preview_ui_width, value_type=int)
    preview_ui_h_int = ParameterValue(preview_ui_height, value_type=int)
    preview_ui_fps_int = ParameterValue(preview_ui_fps, value_type=int)
    cam0_orientation_int = ParameterValue(cam0_orientation, value_type=int)
    cam1_orientation_int = ParameterValue(cam1_orientation, value_type=int)
    preview_source_applies_orientation_bool = ParameterValue(preview_source_applies_orientation, value_type=bool)
    ui_fps_int = ParameterValue(ui_fps, value_type=int)
    trajectory_sample_rate_hz_float = ParameterValue(trajectory_sample_rate_hz, value_type=float)
    trajectory_window_ms_float = ParameterValue(trajectory_window_ms, value_type=float)
    write_trajectory_csv_bool = ParameterValue(write_trajectory_csv, value_type=bool)
    enable_motion_deblur_bool = ParameterValue(enable_motion_deblur, value_type=bool)
    deblur_exposure_ms_float = ParameterValue(deblur_exposure_ms, value_type=float)
    deblur_exposure_time_us_int = ParameterValue(deblur_exposure_time_us, value_type=int)
    deblur_fov_deg_float = ParameterValue(deblur_fov_deg, value_type=float)
    deblur_strength_float = ParameterValue(deblur_strength, value_type=float)
    deblur_min_kernel_px_float = ParameterValue(deblur_min_kernel_px, value_type=float)
    deblur_max_kernel_px_int = ParameterValue(deblur_max_kernel_px, value_type=int)
    deblur_iterations_int = ParameterValue(deblur_iterations, value_type=int)
    deblur_wiener_snr_float = ParameterValue(deblur_wiener_snr, value_type=float)
    deblur_imu_to_cam_yaw_deg_float = ParameterValue(deblur_imu_to_cam_yaw_deg, value_type=float)
    deblur_use_translation_bool = ParameterValue(deblur_use_translation, value_type=bool)
    deblur_assumed_depth_m_float = ParameterValue(deblur_assumed_depth_m, value_type=float)
    deblur_max_odom_age_ms_float = ParameterValue(deblur_max_odom_age_ms, value_type=float)
    deblur_allow_nearest_fallback_bool = ParameterValue(deblur_allow_nearest_fallback, value_type=bool)
    deblur_max_integration_gap_ms_float = ParameterValue(deblur_max_integration_gap_ms, value_type=float)
    deblur_require_time_reference_bool = ParameterValue(deblur_require_time_reference, value_type=bool)
    deblur_max_time_reference_age_ms_float = ParameterValue(deblur_max_time_reference_age_ms, value_type=float)
    ui_cam0_topic = PythonExpression([
        "'/cam1/preview/image_raw' if '", swap_preview_feeds, "' == 'true' else '/cam0/preview/image_raw'"
    ])
    ui_cam1_topic = PythonExpression([
        "'/cam0/preview/image_raw' if '", swap_preview_feeds, "' == 'true' else '/cam1/preview/image_raw'"
    ])

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
            'orientation': cam0_orientation_int,
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
            'orientation': cam1_orientation_int,
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
            'orientation': cam0_orientation_int,
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
            'orientation': cam1_orientation_int,
            'use_node_time': False,
        }],
        respawn=True,
        respawn_delay=6.0,
        condition=cam_condition_respawn,
    )

    # --- GNSS publisher from local gpsd
    gpsd_client = ComposableNode(
        package='gpsd_client',
        plugin='gpsd_client::GPSDClientComponent',
        name='gpsd_client',
        parameters=[{
            'host': gpsd_host,
            'port': gpsd_port_int,
        }],
        remappings=[
            ('fix', '/fix'),
            ('extended_fix', '/extended_fix'),
        ],
    )
    gpsd_container = ComposableNodeContainer(
        name='gps_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[gpsd_client],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", start_gpsd_client, "' == 'true' and '",
                use_gpsd_json_bridge, "' == 'false'"
            ])
        ),
    )

    gpsd_json_bridge = Node(
        package='subsea_bringup',
        executable='gpsd_json_fix_bridge',
        name='gpsd_json_fix_bridge',
        output='screen',
        parameters=[{
            'host': gpsd_host,
            'port': gpsd_port_int,
            'fix_topic': '/fix',
            'frame_id': 'gps',
            'publish_no_fix': False,
        }],
        condition=IfCondition(use_gpsd_json_bridge),
    )

    imu_node = Node(
        package='subsea_bringup',
        executable='bno085_imu_node',
        name='bno085_imu',
        output='screen',
        parameters=[{
            'imu_topic': imu_topic,
            'frame_id': imu_frame_id,
            'rate_hz': imu_rate_hz_float,
            'i2c_address': imu_i2c_address_int,
            'i2c_bus': imu_i2c_bus_int,
            'timestamp_mode': imu_timestamp_mode,
            'enable_rotation': imu_enable_rotation_bool,
            'enable_accel': imu_enable_accel_bool,
            'enable_gyro': imu_enable_gyro_bool,
        }],
        condition=IfCondition(start_imu_node),
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
            # Keep UI alive even if one preview node fails to start.
            'fallback_black_previews': True,
            # Dual-camera rig is expected in field use; avoid transient auto-detect misses.
            'auto_detect_cameras': False,

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
            'trajectory_sample_rate_hz': trajectory_sample_rate_hz_float,
            'trajectory_window_ms': trajectory_window_ms_float,
            'write_trajectory_csv': write_trajectory_csv_bool,
            'enable_motion_deblur': enable_motion_deblur_bool,
            'deblur_exposure_ms': deblur_exposure_ms_float,
            'deblur_exposure_time_us': deblur_exposure_time_us_int,
            'deblur_image_stamp_reference': deblur_image_stamp_reference,
            'deblur_timestamp_source': deblur_timestamp_source,
            'deblur_fov_deg': deblur_fov_deg_float,
            'deblur_strength': deblur_strength_float,
            'deblur_min_kernel_px': deblur_min_kernel_px_float,
            'deblur_max_kernel_px': deblur_max_kernel_px_int,
            'deblur_iterations': deblur_iterations_int,
            'deblur_method': deblur_method,
            'deblur_wiener_snr': deblur_wiener_snr_float,
            'deblur_imu_to_cam_yaw_deg': deblur_imu_to_cam_yaw_deg_float,
            'deblur_use_translation': deblur_use_translation_bool,
            'deblur_assumed_depth_m': deblur_assumed_depth_m_float,
            'deblur_max_odom_age_ms': deblur_max_odom_age_ms_float,
            'deblur_allow_nearest_fallback': deblur_allow_nearest_fallback_bool,
            'deblur_max_integration_gap_ms': deblur_max_integration_gap_ms_float,
            'deblur_require_time_reference': deblur_require_time_reference_bool,
            'deblur_max_time_reference_age_ms': deblur_max_time_reference_age_ms_float,
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
            'cam0_orientation': cam0_orientation_int,
            'cam1_orientation': cam1_orientation_int,
            'preview_source_applies_orientation': preview_source_applies_orientation_bool,
            'preview_relay_enable': True,
            'preview_relay_width': preview_ui_w_int,
            'preview_relay_height': preview_ui_h_int,
            'preview_relay_fps': preview_ui_fps_int,
            'preview_format': preview_format,
            'preview_role': 'viewfinder',
            # Conservative timing reduces "device busy"/broken-pipe startup races.
            'preview_start_stagger_s': 0.7,
            'preview_restart_attempts': 4,
            'preview_restart_delay_s': 0.6,
            'preview_shutdown_timeout_s': 2.5,
            'device_release_timeout_s': 2.5,
            'capture_parallel': True,

            # Capture-stream topics from camera_ros:
            'cam0_namespace': '/cam0',
            'cam1_namespace': '/cam1',
            'cam0_node_name': 'camera',
            'cam1_node_name': 'camera',
            # UI consumes relayed low-load preview topics:
            'ui_cam0_node_name': 'preview',
            'ui_cam1_node_name': 'preview',
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
            'preview_relay_fps': preview_ui_fps_int,
            'cam0_topic': ui_cam0_topic,
            'cam1_topic': ui_cam1_topic,
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
        DeclareLaunchArgument('start_gpsd_client', default_value='true'),
        DeclareLaunchArgument('use_gpsd_json_bridge', default_value='false'),
        DeclareLaunchArgument('gpsd_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('gpsd_port', default_value='2947'),
        DeclareLaunchArgument('start_imu_node', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('imu_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('imu_i2c_address', default_value='74'),
        DeclareLaunchArgument('imu_i2c_bus', default_value='1'),
        DeclareLaunchArgument('imu_timestamp_mode', default_value='read_end'),
        DeclareLaunchArgument('imu_enable_rotation', default_value='true'),
        DeclareLaunchArgument('imu_enable_accel', default_value='true'),
        DeclareLaunchArgument('imu_enable_gyro', default_value='true'),
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
        DeclareLaunchArgument('preview_ui_width', default_value='640'),
        DeclareLaunchArgument('preview_ui_height', default_value='360'),
        DeclareLaunchArgument('preview_ui_fps', default_value='15'),
        DeclareLaunchArgument('preview_format', default_value='BGR888'),
        DeclareLaunchArgument('swap_preview_feeds', default_value='false'),
        DeclareLaunchArgument('cam0_orientation', default_value='0'),
        DeclareLaunchArgument('cam1_orientation', default_value='0'),
        DeclareLaunchArgument('preview_source_applies_orientation', default_value='true'),
        DeclareLaunchArgument('ui_fps', default_value='15'),
        DeclareLaunchArgument('odom_local_topic', default_value='/odometry/local'),
        DeclareLaunchArgument('odom_global_topic', default_value='/odometry/global'),
        DeclareLaunchArgument('capture_debug_topic', default_value='/capture/debug'),
        DeclareLaunchArgument('trajectory_sample_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('trajectory_window_ms', default_value='1000.0'),
        DeclareLaunchArgument('write_trajectory_csv', default_value='true'),
        DeclareLaunchArgument('enable_motion_deblur', default_value='true'),
        DeclareLaunchArgument('deblur_exposure_ms', default_value='8.0'),
        DeclareLaunchArgument('deblur_exposure_time_us', default_value='0'),
        DeclareLaunchArgument('deblur_image_stamp_reference', default_value='midpoint'),
        DeclareLaunchArgument('deblur_timestamp_source', default_value='pps_disciplined_system_clock'),
        DeclareLaunchArgument('deblur_fov_deg', default_value='72.0'),
        DeclareLaunchArgument('deblur_strength', default_value='1.0'),
        DeclareLaunchArgument('deblur_min_kernel_px', default_value='1.2'),
        DeclareLaunchArgument('deblur_max_kernel_px', default_value='31'),
        DeclareLaunchArgument('deblur_iterations', default_value='12'),
        DeclareLaunchArgument('deblur_method', default_value='richardson_lucy',
                              description='Deblur algorithm: richardson_lucy or wiener'),
        DeclareLaunchArgument('deblur_wiener_snr', default_value='40.0',
                              description='Wiener SNR (power ratio, higher=sharper). Only used when deblur_method=wiener'),
        DeclareLaunchArgument('deblur_imu_to_cam_yaw_deg', default_value='0.0',
                              description='Degrees CCW to rotate blur vector from IMU frame to camera image frame'),
        DeclareLaunchArgument('deblur_use_translation', default_value='false',
                              description='Add translational velocity from odometry to blur estimate'),
        DeclareLaunchArgument('deblur_assumed_depth_m', default_value='1.5',
                              description='Assumed scene depth in metres for translational blur (requires deblur_use_translation=true)'),
        DeclareLaunchArgument('deblur_max_odom_age_ms', default_value='200.0',
                              description='Max age of odometry sample used for translational blur'),
        DeclareLaunchArgument('deblur_allow_nearest_fallback', default_value='false'),
        DeclareLaunchArgument('deblur_max_integration_gap_ms', default_value='25.0'),
        DeclareLaunchArgument('deblur_require_time_reference', default_value='true'),
        DeclareLaunchArgument('deblur_max_time_reference_age_ms', default_value='2000.0'),

        *env_actions,

        gpsd_container,
        gpsd_json_bridge,
        imu_node,
        cam0, cam1, cam0_r, cam1_r,
        capture,
        ui,
        localization,
        shutdown_on_ui_exit,
    ])
