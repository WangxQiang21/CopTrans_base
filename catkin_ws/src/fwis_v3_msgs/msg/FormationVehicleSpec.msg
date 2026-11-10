# One vehicle in a rigid formation request. Units: m and rad.
uint32 vehicle_id
geometry_msgs/Point offset_in_formation  # z is always 0; rigid offset r_i in the formation frame
geometry_msgs/Pose2D start_center       # x/y = p_i(0); theta = body heading theta_i, not psi_F
geometry_msgs/Pose2D goal_center        # x/y = rigid goal center; theta is ignored only if goal_heading_free
bool goal_heading_free                  # true frees terminal theta_i only, never the center position
float64 body_length_m
float64 body_width_m
