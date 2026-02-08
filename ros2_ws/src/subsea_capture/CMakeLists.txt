cmake_minimum_required(VERSION 3.8)
project(subsea_capture)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CapturePair.srv"
  DEPENDENCIES builtin_interfaces
)

# Install the Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()