# Geometry-only formation seed. It is not an executable timed trajectory.
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationPrimitive[] primitives
