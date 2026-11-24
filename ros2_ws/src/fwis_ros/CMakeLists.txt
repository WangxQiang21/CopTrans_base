cmake_minimum_required(VERSION 3.8)
project(fwis_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

ament_python_install_package(fwis_ros)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
