cmake_minimum_required(VERSION 3.0.2)
project(fwis_planner_hybrid_astar)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  fwis_common
  fwis_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp fwis_common fwis_msgs nav_msgs geometry_msgs visualization_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/planner.cpp
  src/planner_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/playback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
