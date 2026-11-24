cmake_minimum_required(VERSION 3.8)
project(fwis_v3_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FormationPlanRequest.msg"
  "msg/FormationPlanSeed.msg"
  "msg/FormationPlaybackPlan.msg"
  "msg/FormationExecutionPermit.msg"
  "msg/FormationPrimitive.msg"
  "msg/FormationTimedPoint.msg"
  "msg/FormationTimedTrajectory.msg"
  "msg/FormationVehicleSpec.msg"
  "msg/FormationVehicleTrajectory.msg"
  "msg/ModePrimitive.msg"
  "msg/PlanSeed.msg"
  "msg/PlanSeedCandidates.msg"
  "msg/TimedPoint.msg"
  "msg/TimedTrajectory.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
