# Multi-vehicle formation request. plan_id identifies this request; formation_id
# identifies the fixed offset configuration and is not a request ID.
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start    # x_F, y_F, psi_F
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationVehicleSpec[] vehicles
bool allow_reverse
# A continuation refers to the exact playback-confirmed terminal state.
# Leave both fields empty/zero for a fresh experiment.
uint32 continuation_plan_id
string continuation_model_sha256
