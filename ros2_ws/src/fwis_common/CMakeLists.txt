cmake_minimum_required(VERSION 3.8)
project(fwis_common)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(nav_msgs)

ament_package()
