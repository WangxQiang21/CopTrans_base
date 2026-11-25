cmake_minimum_required(VERSION 3.16)
project(fwis_hybrid_joint_ros)
find_package(ament_cmake REQUIRED)
find_package(fwis_hybrid_joint REQUIRED)
find_package(casadi CONFIG REQUIRED HINTS
  "$ENV{COPTRANS_CASADI_ROOT}/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.coptrans-python/casadi/cmake")
find_package(fwis_v3_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenSSL REQUIRED)
add_library(hybrid_codec SHARED src/codec.cpp src/request.cpp)
target_include_directories(hybrid_codec PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
file(GLOB_RECURSE model_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_hybrid_joint/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_hybrid_joint/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_hybrid_joint/src/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_planning_core/include/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_planning_core/src/collision.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_planning_core/src/geometry.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/request.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../fwis_hybrid_joint/include/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/codec.cpp")
set(model_hashes "CasADi=${casadi_VERSION};compiler=${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION};build=${CMAKE_BUILD_TYPE};flags=${CMAKE_CXX_FLAGS}")
foreach(path IN LISTS model_sources)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${path}")
  file(SHA256 "${path}" part)
  string(APPEND model_hashes "${part}")
endforeach()
string(SHA256 model_version "${model_hashes}")
# Installed SDK headers retain source mtimes. Export the content-derived model
# version so all consumers recompile even when copying a changed header with
# an older timestamp than a previously built object.
target_compile_definitions(hybrid_codec PUBLIC HYBRID_BUILD_DIGEST="${model_version}")
target_compile_features(hybrid_codec PUBLIC cxx_std_17)
target_link_libraries(hybrid_codec fwis_hybrid_joint::fwis_hybrid_joint JsonCpp::JsonCpp OpenSSL::Crypto)
ament_target_dependencies(hybrid_codec fwis_v3_msgs geometry_msgs nav_msgs rclcpp std_msgs)
foreach(role planner executor)
  add_executable(hybrid_joint_${role} src/${role}.cpp)
  target_link_libraries(hybrid_joint_${role} hybrid_codec)
  target_compile_options(hybrid_joint_${role} PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS hybrid_joint_${role} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(TARGETS hybrid_codec EXPORT hybrid_ros_targets DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(hybrid_ros_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(fwis_hybrid_joint fwis_v3_msgs geometry_msgs nav_msgs rclcpp std_msgs jsoncpp OpenSSL)
if(BUILD_TESTING)
  enable_testing()
  add_executable(hybrid_codec_test test/test_codec.cpp)
  target_link_libraries(hybrid_codec_test hybrid_codec)
  add_test(NAME hybrid_authoritative_codec COMMAND hybrid_codec_test)
endif()
ament_package()
