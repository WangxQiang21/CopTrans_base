cmake_minimum_required(VERSION 3.0.2)
project(fwis_map_server)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/gen_map.py
  scripts/map_publisher.py
  scripts/scene_hints.py
  scripts/parking_lines.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
