# Auxiliary search output for the optimizer. These primitives are provisional
# initializers and must pass the shared action-policy repair and full audit;
# they are never published as the primary /plan/seed path.
std_msgs/Header header
uint32 plan_id
fwis_v3_msgs/ModePrimitive[] proposal_primitives
string proposal_source
