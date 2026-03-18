from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Force software rendering for better remote/container compatibility.
    env_no_mitshm = SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1")
    env_qt_opengl = SetEnvironmentVariable(name="QT_OPENGL", value="software")
    env_qt_xcb = SetEnvironmentVariable(name="QT_XCB_GL_INTEGRATION", value="none")
    env_libgl_sw = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")

    mock_cam = Node(
        package="subsea_mock",
        executable="mock_camera_publisher",
        output="screen",
        parameters=[
            {"width": 960, "height": 540, "fps": 15},
        ],
    )

    mock_cap = Node(
        package="subsea_mock",
        executable="mock_capture_service",
        output="screen",
        parameters=[
            {"width": 1280, "height": 720, "default_quality": 90},
        ],
    )

    ui = Node(
        package="subsea_ui",
        executable="ui",
        output="screen",
    )

    return LaunchDescription([
        env_no_mitshm,
        env_qt_opengl,
        env_qt_xcb,
        env_libgl_sw,
        mock_cam,
        mock_cap,
        ui,
    ])
