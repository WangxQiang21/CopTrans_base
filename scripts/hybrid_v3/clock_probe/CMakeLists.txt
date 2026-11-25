cmake_minimum_required(VERSION 3.16)
project(coptrans_clock_take_probe LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(clock_take SHARED clock_take.cpp)
target_compile_features(clock_take PRIVATE cxx_std_17)
ament_target_dependencies(clock_take rcl rosgraph_msgs nav_msgs)
target_link_libraries(clock_take ${CMAKE_DL_LIBS})
