cmake_minimum_required(VERSION 3.0.2)
project(fwis_v3_playback)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs visualization_msgs fwis_v3_msgs std_msgs tf2_ros)
catkin_package(CATKIN_DEPENDS rospy geometry_msgs visualization_msgs fwis_v3_msgs std_msgs tf2_ros)
catkin_install_python(PROGRAMS
  scripts/playback.py
  scripts/formation_playback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
