cmake_minimum_required(VERSION 3.8)
project(fwis_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_experiment DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(formation_route test/test_formation_route.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(interactive_formation test/test_interactive_formation.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(experiment_log test/test_experiment_log.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(stop_observation test/test_stop_observation.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(assembly_plan test/test_assembly_plan.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_scene_variant test/test_scene_variant.py)
endif()
ament_package()
