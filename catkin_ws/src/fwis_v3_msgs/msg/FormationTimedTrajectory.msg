# One common time grid for every vehicle. formation_poses[i].theta is psi_F.
# trajectories[k].points[j].time_from_start must equal times_from_start[j].
std_msgs/Header header
uint32 plan_id
string formation_id
fwis_v3_msgs/FormationVehicleSpec[] vehicles
geometry_msgs/Pose2D[] formation_poses
duration[] times_from_start
fwis_v3_msgs/FormationVehicleTrajectory[] trajectories
