cmake_minimum_required(VERSION 3.12)
project(fwis_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Gui Widgets)

add_library(tracking_hud SHARED src/tracking_display.cpp src/tracking_hud.cpp src/planning_panel.cpp)
target_compile_features(tracking_hud PRIVATE cxx_std_17)
target_compile_options(tracking_hud PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tracking_hud
  nav_msgs pluginlib rclcpp rviz_common rviz_default_plugins rviz_rendering
  std_msgs visualization_msgs)
target_link_libraries(tracking_hud Qt5::Gui Qt5::Widgets rviz_ogre_vendor::OgreOverlay)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)
install(TARGETS tracking_hud LIBRARY DESTINATION lib)
ament_package()
