cmake_minimum_required(VERSION 3.8)
project(fwis_backends)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fwis_planning_core REQUIRED)
set(planner_dependencies
  fwis_common fwis_ros fwis_v3_msgs geometry_msgs nav_msgs rclcpp std_msgs
  tf2 tf2_geometry_msgs visualization_msgs
)
foreach(dependency IN LISTS planner_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

foreach(backend IN ITEMS single_multimodal_hybrid_astar single_constant_primitive single_continuous_mode_path)
  add_executable(${backend}_planner ${backend}/planner_node.cpp)
  target_compile_features(${backend}_planner PRIVATE cxx_std_17)
  target_compile_options(${backend}_planner PRIVATE -O2 -Wall -Wextra)
  ament_target_dependencies(${backend}_planner ${planner_dependencies})
  install(TARGETS ${backend}_planner DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(multi_4wis_legacy_planner
  multi_4wis/formation_planner_node.cpp multi_4wis/formation_search.cpp
)
target_compile_features(multi_4wis_legacy_planner PRIVATE cxx_std_17)
target_compile_options(multi_4wis_legacy_planner PRIVATE -O2 -Wall -Wextra)
target_include_directories(multi_4wis_legacy_planner PRIVATE multi_4wis/include)
ament_target_dependencies(multi_4wis_legacy_planner ${planner_dependencies})
install(TARGETS multi_4wis_legacy_planner DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(fwis_backends)

find_package(pybind11 CONFIG QUIET)
if(pybind11_FOUND)
  find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  pybind11_add_module(_formation_collision_native MODULE multi_4wis/formation_collision_bindings.cpp)
  target_link_libraries(_formation_collision_native PRIVATE fwis_planning_core::fwis_planning_core)
  target_compile_features(_formation_collision_native PRIVATE cxx_std_17)
  # Preserve the separating-axis comparisons at exact contact.
  target_compile_options(_formation_collision_native PRIVATE -O3 -Wall -Wextra -ffp-contract=off)
  install(TARGETS _formation_collision_native
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/fwis_backends/multi_4wis")
else()
  message(STATUS "pybind11 was not found; using the Python formation collision kernel")
endif()

foreach(backend IN ITEMS single_multimodal_hybrid_astar single_constant_primitive single_continuous_mode_path)
  install(PROGRAMS ${backend}/scripts/optimizer_node.py
    DESTINATION lib/${PROJECT_NAME} RENAME ${backend}_optimizer)
  install(PROGRAMS ${backend}/scripts/playback.py
    DESTINATION lib/${PROJECT_NAME} RENAME ${backend}_playback)
  install(DIRECTORY ${backend}/config/ DESTINATION share/${PROJECT_NAME}/config/${backend})
endforeach()

install(PROGRAMS single_continuous_mode_path/scripts/single_state_lattice_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  multi_4wis/scripts/formation_uniform_planner_node.py
  multi_4wis/scripts/formation_optimizer_node.py
  multi_4wis/scripts/formation_playback.py
  multi_4wis/scripts/formation_goal_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY multi_4wis/config/ DESTINATION share/${PROJECT_NAME}/config/multi_4wis)
ament_package()
