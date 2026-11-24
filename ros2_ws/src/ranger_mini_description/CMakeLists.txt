cmake_minimum_required(VERSION 3.12)
project(ranger_mini_description)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Preserve the original compressed archive for the 109 MB Collada source.
set(base_archive "${CMAKE_CURRENT_SOURCE_DIR}/meshes/ranger_base.zip")
set(base_mesh "${CMAKE_CURRENT_BINARY_DIR}/ranger_base.dae")
add_custom_command(OUTPUT "${base_mesh}"
  COMMAND "${CMAKE_COMMAND}" -E tar xf "${base_archive}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  DEPENDS "${base_archive}"
  VERBATIM)
add_custom_target(ranger_mini_base_mesh ALL DEPENDS "${base_mesh}")
install(FILES "${base_mesh}" meshes/fl.dae meshes/fr.dae DESTINATION share/${PROJECT_NAME}/meshes)

set(painted_dir "${CMAKE_CURRENT_BINARY_DIR}/painted")
set(painted_meshes
  "${painted_dir}/ranger_base.dae"
  "${painted_dir}/fl.dae"
  "${painted_dir}/fr.dae"
  "${painted_dir}/shell_white.dae"
  "${painted_dir}/shell_orange.dae"
  "${painted_dir}/shell_blue.dae")
set(paint_script "${CMAKE_CURRENT_SOURCE_DIR}/meshes/paint_meshes.py")
set(left_wheel "${CMAKE_CURRENT_SOURCE_DIR}/meshes/fl.dae")
set(right_wheel "${CMAKE_CURRENT_SOURCE_DIR}/meshes/fr.dae")
add_custom_command(
  OUTPUT ${painted_meshes}
  COMMAND "${Python3_EXECUTABLE}" "${paint_script}" "${painted_dir}"
          "${base_mesh}" "${left_wheel}" "${right_wheel}"
  DEPENDS "${paint_script}" "${base_mesh}" "${left_wheel}" "${right_wheel}"
  VERBATIM)
add_custom_target(ranger_mini_painted_meshes ALL DEPENDS ${painted_meshes})
add_dependencies(ranger_mini_painted_meshes ranger_mini_base_mesh)
install(FILES ${painted_meshes} DESTINATION share/${PROJECT_NAME}/meshes/painted)

install(DIRECTORY urdf rviz DESTINATION share/${PROJECT_NAME})
ament_package()
