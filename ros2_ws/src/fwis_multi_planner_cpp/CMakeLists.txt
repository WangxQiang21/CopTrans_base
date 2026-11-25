cmake_minimum_required(VERSION 3.16)
project(fwis_multi_planner_cpp LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(CTest)
include(GNUInstallDirs)
find_package(ament_cmake QUIET)
find_package(fwis_planning_core REQUIRED)

add_library(fwis_multi_frontier STATIC src/frontier.cpp src/successors.cpp)
set_target_properties(fwis_multi_frontier PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(fwis_multi_frontier PUBLIC cxx_std_17)
target_compile_options(fwis_multi_frontier PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
# Match the independent Python trigonometric evaluations used to seed lattice controls.
set_source_files_properties(src/successors.cpp PROPERTIES COMPILE_OPTIONS "-fno-builtin-sin;-fno-builtin-cos")
target_include_directories(fwis_multi_frontier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(fwis_multi_frontier PUBLIC fwis_planning_core::fwis_planning_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS fwis_multi_frontier EXPORT export_fwis_multi_frontier
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

find_package(pybind11 CONFIG QUIET)
if(pybind11_FOUND)
  find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
  pybind11_add_module(fwis_multi_frontier_native MODULE NO_EXTRAS src/python_bindings.cpp)
  target_compile_options(fwis_multi_frontier_native PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(fwis_multi_frontier_native PRIVATE fwis_multi_frontier)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_python REQUIRED)
    ament_get_python_install_dir(PYTHON_INSTALL_DIR)
      # Native extension targets are installed directly, so register the
      # Python path hook that ament_python_install_package normally adds.
      set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_pythonpath
        "prepend-non-duplicate;PYTHONPATH;${PYTHON_INSTALL_DIR}")
      ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_PYTHONPATH}")
    install(TARGETS fwis_multi_frontier_native LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
  endif()
else()
  message(STATUS "pybind11 unavailable; the optional C++ frontier cannot be selected from Python")
endif()

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(fwis_multi_frontier_tests test/test_frontier.cpp)
  target_compile_options(fwis_multi_frontier_tests PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(fwis_multi_frontier_tests PRIVATE fwis_multi_frontier GTest::gtest_main)
  add_test(NAME fwis_multi_frontier_test COMMAND fwis_multi_frontier_tests)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_fwis_multi_frontier HAS_LIBRARY_TARGET)
  ament_export_dependencies(fwis_planning_core)
  ament_package()
endif()
