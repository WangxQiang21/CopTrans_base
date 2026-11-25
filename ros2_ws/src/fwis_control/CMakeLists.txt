cmake_minimum_required(VERSION 3.8)
project(fwis_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fwis_v3_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(fwis_tracking_core
  src/tracking_model.cpp
  src/tracking_trajectory.cpp
  src/tracking_execution.cpp)
target_compile_features(fwis_tracking_core PUBLIC cxx_std_17)
target_include_directories(fwis_tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(fwis_tracking_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ranger_tracking_controller_cpp src/tracking_controller_node.cpp)
target_compile_options(ranger_tracking_controller_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ranger_tracking_controller_cpp fwis_tracking_core Threads::Threads yaml-cpp)
ament_target_dependencies(ranger_tracking_controller_cpp
  ament_index_cpp fwis_v3_msgs geometry_msgs nav_msgs rcl rclcpp std_msgs)
install(TARGETS ranger_tracking_controller_cpp DESTINATION lib/${PROJECT_NAME})
install(TARGETS fwis_tracking_core EXPORT fwis_tracking_core_targets)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(fwis_tracking_core_targets HAS_LIBRARY_TARGET)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ranger_tracking_controller DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(tracking_replay test/tracking_replay.cpp)
  target_link_libraries(tracking_replay PRIVATE fwis_tracking_core)
  ament_add_pytest_test(tracking_cpp_parity test/test_tracking_cpp_parity.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    ENV "TRACKING_CPP_REPLAY=$<TARGET_FILE:tracking_replay>")

  ament_add_pytest_test(control_timing test/test_control_timing.py)
  ament_add_pytest_test(tracking_core test/test_tracking_core.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
endif()
ament_package()
