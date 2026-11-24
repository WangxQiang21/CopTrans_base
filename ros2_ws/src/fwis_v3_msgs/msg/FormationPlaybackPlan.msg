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
