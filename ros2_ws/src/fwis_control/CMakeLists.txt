cmake_minimum_required(VERSION 3.8)
project(fwis_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fwis_v3_msgs REQUIRED)
find_package(fwis_hybrid_joint_ros REQUIRED)
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
target_link_libraries(ranger_tracking_controller_cpp fwis_tracking_core Threads::Threads yaml-cpp fwis_hybrid_joint_ros::hybrid_codec)
ament_target_dependencies(ranger_tracking_controller_cpp
  ament_index_cpp fwis_v3_msgs geometry_msgs nav_msgs rcl rclcpp std_msgs)
install(TARGETS ranger_tracking_controller_cpp DESTINATION lib/${PROJECT_NAME})

# Used only by the opt-in Gazebo timing probe and its existing ros_gz_bridge process.
add_library(coptrans_bridge_take_trace SHARED test/trace_bridge_take.cpp)
target_compile_features(coptrans_bridge_take_trace PRIVATE cxx_std_17)
target_compile_options(coptrans_bridge_take_trace PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(coptrans_bridge_take_trace ${CMAKE_DL_LIBS})
ament_target_dependencies(coptrans_bridge_take_trace geometry_msgs rcl)
install(TARGETS coptrans_bridge_take_trace DESTINATION lib/${PROJECT_NAME})

install(TARGETS fwis_tracking_core EXPORT fwis_tracking_core_targets)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(fwis_tracking_core_targets HAS_LIBRARY_TARGET)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ranger_tracking_controller DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(fwis_hybrid_joint REQUIRED)
  add_executable(hybrid_deadlines test/hybrid_deadlines.cpp)
  target_link_libraries(hybrid_deadlines PRIVATE fwis_tracking_core
    fwis_hybrid_joint::fwis_hybrid_joint Threads::Threads)
  add_executable(tracking_replay test/tracking_replay.cpp)
  target_link_libraries(tracking_replay PRIVATE fwis_tracking_core)
  ament_add_pytest_test(tracking_cpp_parity test/test_tracking_cpp_parity.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    ENV "TRACKING_CPP_REPLAY=$<TARGET_FILE:tracking_replay>")

  add_executable(native_oblique_replay test/native_oblique_replay.cpp)
  target_link_libraries(native_oblique_replay PRIVATE fwis_tracking_core fwis_hybrid_joint_ros::hybrid_codec)
  ament_add_pytest_test(native_oblique_controller_parity test/test_native_oblique_replay.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    ENV "TRACKING_NATIVE_OBLIQUE=$<TARGET_FILE:native_oblique_replay>")

  add_executable(solved_oblique_replay test/solved_oblique_replay.cpp)
  target_link_libraries(solved_oblique_replay PRIVATE fwis_tracking_core fwis_hybrid_joint_ros::hybrid_codec)
  add_test(NAME solved_oblique_controllers COMMAND solved_oblique_replay)
  set_tests_properties(solved_oblique_controllers PROPERTIES ENVIRONMENT "OPENBLAS_NUM_THREADS=1;OMP_NUM_THREADS=1")

  ament_add_pytest_test(control_timing test/test_control_timing.py)
  add_executable(control_affinity_tests test/control_affinity_tests.cpp)
  target_compile_features(control_affinity_tests PRIVATE cxx_std_17)
  target_link_libraries(control_affinity_tests Threads::Threads)
  ament_target_dependencies(control_affinity_tests rclcpp)
  add_test(NAME control_affinity_and_thread_counters COMMAND control_affinity_tests)
  set_tests_properties(control_affinity_and_thread_counters PROPERTIES SKIP_RETURN_CODE 77)
  ament_add_pytest_test(tracking_core test/test_tracking_core.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
endif()
ament_package()
