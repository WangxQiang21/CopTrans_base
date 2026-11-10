# Multi-vehicle formation request. plan_id identifies this request; formation_id
# identifies the fixed offset configuration and is not a request ID.
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start    # x_F, y_F, psi_F
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationVehicleSpec[] vehicles
bool allow_reverse
