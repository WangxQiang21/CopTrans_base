# 消息定义对照

单车五个消息在四个源提交中逐字节一致。多车源分支增加下列八个 Formation 消息。原始字段顺序、常量、注释和单位见下表。ROS2 用 `builtin_interfaces/Duration` 表达原有的 duration，Header 使用 ROS2 标准类型，时间分辨率仍为纳秒。

## FormationPlanRequest.msg

```text
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
```

## FormationPlanSeed.msg

```text
# Geometry-only formation seed. It is not an executable timed trajectory.
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationPrimitive[] primitives
```

## FormationPlaybackPlan.msg

```text
# Atomic V0.2 playback envelope.  The embedded trajectory is a sample of the
# authoritative versioned model in model_json; playback must re-derive every
# display frame from that model with FR02 analytic sampling, not from linear
# interpolation of the sample points.  model_sha256 is UniformTimedPlan's
# versioned semantic digest of model_json.
#
# Only the uniform_v02 backend publishes this message.  Old message definitions
# remain unchanged; this adds one topic, not a second planner.
std_msgs/Header header
uint32 plan_id
string formation_id
string backend                 # must be "uniform_v02"
string semantic_version        # must be "uniform_playback_plan/v1"
string model_version           # UniformTimedPlan model version
string model_sha256            # digest of model_json
string model_json              # complete versioned UniformTimedPlan model
string certification           # copied from the model; informational only
fwis_v3_msgs/FormationTimedTrajectory trajectory
# Optional sealed formation-corridor binding for the three_vehicle_union_v1
# scope (canonical JSON object; empty for f_only and uniform_v02 plans).  It
# travels inside the atomic envelope so the receiver can re-verify the
# template, corridor chain and continuous certificate against this model
# without a second channel or a message-ordering race.
string formation_binding_json
```

## FormationPrimitive.msg

```text
# Formation-reference primitive, NOT a per-vehicle A/O/S mode primitive.
# All coordinates and angles are expressed in the current formation frame
# (x along psi_F, y left of psi_F) before the primitive is integrated.
uint8 TRANSLATION=0
uint8 ROTATION=1
uint8 COMPOSITE=2
uint8 geometry_type

# TRANSLATION: formation-reference displacement (dx, dy) in metres.
# psi_F is unchanged; every vehicle center translates by the same world vector.
float64 dx
float64 dy

# ROTATION: formation-reference angle dpsi in radians about the explicit
# pivot (pivot_x, pivot_y). The pivot is part of the contract, never hidden in
# dx/dy. A rigid formation rotation is only a geometry candidate, not per-vehicle
# Spin feasibility.
float64 dpsi
float64 pivot_x
float64 pivot_y
```

## FormationTimedPoint.msg

```text
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
```

## FormationTimedTrajectory.msg

```text
# One common time grid for every vehicle. formation_poses[i].theta is psi_F.
# trajectories[k].points[j].time_from_start must equal times_from_start[j].
std_msgs/Header header
uint32 plan_id
string formation_id
fwis_v3_msgs/FormationVehicleSpec[] vehicles
geometry_msgs/Pose2D[] formation_poses
duration[] times_from_start
fwis_v3_msgs/FormationVehicleTrajectory[] trajectories
```

## FormationVehicleSpec.msg

```text
# One vehicle in a rigid formation request. Units: m and rad.
uint32 vehicle_id
geometry_msgs/Point offset_in_formation  # z is always 0; rigid offset r_i in the formation frame
geometry_msgs/Pose2D start_center       # x/y = p_i(0); theta = body heading theta_i, not psi_F
geometry_msgs/Pose2D goal_center        # x/y = rigid goal center; theta is ignored only if goal_heading_free
bool goal_heading_free                  # true frees terminal theta_i only, never the center position
float64 body_length_m
float64 body_width_m
```

## FormationVehicleTrajectory.msg

```text
uint32 vehicle_id
fwis_v3_msgs/FormationTimedPoint[] points
```

## ModePrimitive.msg

```text
uint8 ACKERMANN=0
uint8 OBLIQUE=1
uint8 SPIN=2
uint8 mode
float64 amount
float64 control
```

## PlanSeed.msg

```text
std_msgs/Header header
uint32 plan_id
geometry_msgs/Pose2D start_center
geometry_msgs/Pose2D goal_center
fwis_v3_msgs/ModePrimitive[] primitives
```

## PlanSeedCandidates.msg

```text
# Auxiliary search output for the optimizer. These primitives are provisional
# initializers and must pass the shared action-policy repair and full audit;
# they are never published as the primary /plan/seed path.
std_msgs/Header header
uint32 plan_id
fwis_v3_msgs/ModePrimitive[] proposal_primitives
string proposal_source
```

## TimedPoint.msg

```text
duration time_from_start
geometry_msgs/Pose2D center_pose
float64 v
uint8 ACKERMANN=0
uint8 OBLIQUE=1
uint8 SPIN=2
uint8 mode
```

## TimedTrajectory.msg

```text
std_msgs/Header header
uint32 plan_id
fwis_v3_msgs/TimedPoint[] points
```
