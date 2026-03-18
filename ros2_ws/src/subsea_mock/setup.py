from setuptools import setup
import os
from glob import glob

package_name = "subsea_mock"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="subseascanning",
    maintainer_email="",
    description="Mock camera publisher + capture service for macOS simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mock_camera_publisher = subsea_mock.mock_camera_publisher:main",
            "mock_capture_service = subsea_mock.mock_capture_service:main",
        ],
    },
)
