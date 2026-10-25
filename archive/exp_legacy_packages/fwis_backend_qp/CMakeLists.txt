cmake_minimum_required(VERSION 3.0.2)
project(fwis_backend_qp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  fwis_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy nav_msgs geometry_msgs fwis_msgs
)

catkin_install_python(PROGRAMS
  scripts/velocity_qp_node.py
  scripts/safe_corridor_node.py
  scripts/casadi_safe_corridor_backend.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
