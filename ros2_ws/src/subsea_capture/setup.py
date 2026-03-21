from setuptools import setup

package_name = 'subsea_capture'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subseascanning',
    maintainer_email='',
    description='Dual-camera capture service (stream-synced + rpicam-still fallback)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'capture_service = subsea_capture.capture_service:main',
        ],
    },
)
