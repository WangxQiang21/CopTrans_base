cmake_minimum_required(VERSION 3.16)
project(fwis_planning_core LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(CTest)
include(GNUInstallDirs)
find_package(ament_cmake QUIET)
option(FWIS_PLANNING_CORE_PYTHON "Build the optional NumPy batch binding" ON)
option(FWIS_PLANNING_CORE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_library(fwis_planning_core STATIC src/geometry.cpp src/collision.cpp src/trajectory.cpp)
add_library(fwis_planning_core::fwis_planning_core ALIAS fwis_planning_core)
set_target_properties(fwis_planning_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(fwis_planning_core PUBLIC cxx_std_17)
target_compile_options(fwis_planning_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
# Preserve Python's independent sin/cos rounding near the straight/arc threshold.
set_source_files_properties(src/geometry.cpp PROPERTIES COMPILE_OPTIONS "-fno-builtin-sin;-fno-builtin-cos")
target_include_directories(fwis_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

if(FWIS_PLANNING_CORE_SANITIZERS)
  target_compile_options(fwis_planning_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(fwis_planning_core PUBLIC -fsanitize=address,undefined)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS fwis_planning_core EXPORT export_fwis_planning_core
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(FWIS_PLANNING_CORE_PYTHON)
  find_package(pybind11 CONFIG QUIET)
  if(pybind11_FOUND)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    pybind11_add_module(fwis_planning_core_native MODULE NO_EXTRAS src/python_bindings.cpp)
    target_link_libraries(fwis_planning_core_native PRIVATE fwis_planning_core)
    target_compile_options(fwis_planning_core_native PRIVATE -Wall -Wextra -Wpedantic)
    if(ament_cmake_FOUND)
      find_package(ament_cmake_python REQUIRED)
      ament_get_python_install_dir(PYTHON_INSTALL_DIR)
      # Native extension targets are installed directly, so register the
      # Python path hook that ament_python_install_package normally adds.
      set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_pythonpath
        "prepend-non-duplicate;PYTHONPATH;${PYTHON_INSTALL_DIR}")
      ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_PYTHONPATH}")
      install(TARGETS fwis_planning_core_native LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
    endif()
  else()
    message(STATUS "pybind11 unavailable; building only the pure C++ planning core")
  endif()
endif()

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(fwis_planning_core_tests test/test_planning_core.cpp)
  target_link_libraries(fwis_planning_core_tests PRIVATE fwis_planning_core GTest::gtest_main)
  target_compile_options(fwis_planning_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME fwis_planning_core_test COMMAND fwis_planning_core_tests)
endif()

find_package(benchmark CONFIG QUIET)
if(benchmark_FOUND)
  add_executable(fwis_planning_core_benchmark benchmark/planning_core_benchmark.cpp)
  target_link_libraries(fwis_planning_core_benchmark PRIVATE fwis_planning_core benchmark::benchmark)
  target_compile_options(fwis_planning_core_benchmark PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_fwis_planning_core HAS_LIBRARY_TARGET)
  ament_package()
else()
  install(EXPORT export_fwis_planning_core NAMESPACE fwis_planning_core::
    FILE fwis_planning_coreConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fwis_planning_core)
endif()
