Header header
geometry_msgs/Pose2D[] pose
float64[] v
float64[] omega
uint8[] mode      # 0=Ackermann, 1=Lateral, 2=Diagonal/Crab, 3=Spin
float64[] phi     # diagonal heading angle (rad), valid when mode==2
float64[] d_switch
