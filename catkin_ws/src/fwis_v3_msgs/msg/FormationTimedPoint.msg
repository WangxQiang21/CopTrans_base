# One common-time sample for one vehicle.
# center_pose.theta is the body heading theta_i, not formation heading psi_F.
duration time_from_start
geometry_msgs/Pose2D center_pose
float64 center_speed_mps
float64 center_yaw_rate_radps
uint8 ACKERMANN=0
uint8 OBLIQUE=1
uint8 SPIN=2
uint8 mode
float64 mode_control
