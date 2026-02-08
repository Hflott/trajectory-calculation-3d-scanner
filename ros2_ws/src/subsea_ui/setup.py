from setuptools import setup

package_name = 'subsea_ui'

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
    description='Touch UI for dual camera preview + capture + result display',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ui = subsea_ui.ui:main',
        ],
    },
)
