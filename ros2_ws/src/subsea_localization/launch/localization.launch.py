import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("subsea_localization")
    ekf_local_yaml = os.path.join(pkg_share, "config", "ekf_local.yaml")
    ekf_global_yaml = os.path.join(pkg_share, "config", "ekf_global.yaml")
    navsat_yaml = os.path.join(pkg_share, "config", "navsat_transform.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_topic = LaunchConfiguration("imu_topic")
    gps_fix_topic = LaunchConfiguration("gps_fix_topic")
    odom_input_topic = LaunchConfiguration("odom_input_topic")
    odom_local_topic = LaunchConfiguration("odom_local_topic")
    odom_global_topic = LaunchConfiguration("odom_global_topic")
    odom_gps_topic = LaunchConfiguration("odom_gps_topic")
    filtered_gps_topic = LaunchConfiguration("filtered_gps_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_link_frame = LaunchConfiguration("base_link_frame")
    magnetic_declination_radians = LaunchConfiguration("magnetic_declination_radians")
    yaw_offset = LaunchConfiguration("yaw_offset")
    zero_altitude = LaunchConfiguration("zero_altitude")

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[
            ekf_local_yaml,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_link_frame,
                "world_frame": odom_frame,
                "imu0": imu_topic,
                "odom0": odom_input_topic,
            },
        ],
        remappings=[("odometry/filtered", odom_local_topic)],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            navsat_yaml,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "magnetic_declination_radians": ParameterValue(magnetic_declination_radians, value_type=float),
                "yaw_offset": ParameterValue(yaw_offset, value_type=float),
                "zero_altitude": ParameterValue(zero_altitude, value_type=bool),
            },
        ],
        remappings=[
            ("imu", imu_topic),
            ("gps/fix", gps_fix_topic),
            ("odometry/filtered", odom_local_topic),
            ("odometry/gps", odom_gps_topic),
            ("gps/filtered", filtered_gps_topic),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[
            ekf_global_yaml,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_link_frame,
                "world_frame": map_frame,
                "imu0": imu_topic,
                "odom0": odom_local_topic,
                "odom1": odom_gps_topic,
            },
        ],
        remappings=[("odometry/filtered", odom_global_topic)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("gps_fix_topic", default_value="/fix"),
            DeclareLaunchArgument("odom_input_topic", default_value="/odometry/wheel"),
            DeclareLaunchArgument("odom_local_topic", default_value="/odometry/local"),
            DeclareLaunchArgument("odom_global_topic", default_value="/odometry/global"),
            DeclareLaunchArgument("odom_gps_topic", default_value="/odometry/gps"),
            DeclareLaunchArgument("filtered_gps_topic", default_value="/gps/filtered"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_link_frame", default_value="base_link"),
            DeclareLaunchArgument("magnetic_declination_radians", default_value="0.0"),
            DeclareLaunchArgument("yaw_offset", default_value="0.0"),
            DeclareLaunchArgument("zero_altitude", default_value="false"),
            ekf_local,
            navsat,
            ekf_global,
        ]
    )
