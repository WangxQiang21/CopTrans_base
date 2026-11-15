"""FR04-R1 recheck: a final rebuild cannot escape the original request.

Adapted from Codex's independent probe of 2026-11-14.  The fault injection is
unchanged: replace only the second trajectory reconstruction return with a
real, self-feasible FR01/FR02 plan from (0,0,0) to (1,0,0).  Unlike the first
version, this probe never reads ``result.plan.start_qf`` or
``result.plan.goal_qf`` unconditionally: an expected rejection has no plan.

Run from the worktree root with::

    OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 \
        evaluation/formation_refactor_v02/FR04/rework_01/codex_probe/\
        adapted_independent_probes.py
"""
from pathlib import Path
import json
import sys
from unittest.mock import patch

ROOT = Path.cwd()
sys.path[:0] = [str(ROOT / "tests"),
                str(ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src")]
import test_uniform_pipeline as t
import fwis_v3_optimizer.uniform_pipeline as p
from fwis_v3_optimizer.uniform_motion import a_straight_edge
from fwis_v3_optimizer.uniform_quality import (
    FEASIBLE,
    UniformAuditRequest,
    audit_uniform_timed_plan,
)
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan

OUT = Path(__file__).parent
rows = []


def plan_fields(plan):
    if plan is None:
        return {"plan_present": False, "start": None, "goal": None,
                "plan_id": None}
    return {
        "plan_present": True,
        "plan_id": plan.plan_id,
        "start": [plan.start_qf.x, plan.start_qf.y, plan.start_qf.psi],
        "goal": [plan.goal_qf.x, plan.goal_qf.y, plan.goal_qf.psi],
    }


def result_fields(result):
    record = {
        "status": result.status,
        "feasible": result.feasible,
        "executable": result.executable,
        "reason": result.reason,
        "failure_layer": result.first_failure_layer,
        "search_edge_count": (0 if result.search is None
                              else len(result.search.edges)),
        "evaluation_status": (None if result.evaluation is None
                              else result.evaluation.status),
    }
    record.update(plan_fields(result.plan))
    return record


# A valid but unrelated final plan is injected exactly at reconstruction
# output.  The real request is (10,10,0) -> (13,10,0), ID 401.
req = t.make_request(t.FormationPose(13, 10, 0), plan_id=401)
other = a_straight_edge(
    req.model, t.FormationPose(0, 0, 0), req.start_headings, (1, 0),
    {vehicle_id: 1 for vehicle_id in t.VEHICLE_IDS}, allow_reverse=True)
wrong = UniformTimedPlan.from_edges(
    (other,), req.vehicle_shapes, plan_id=req.plan_id,
    formation_id=req.formation_id, model=req.model)
wrong_self = audit_uniform_timed_plan(
    UniformAuditRequest.from_plan(wrong), wrong, wrong.sample(), req.grid)
real_from_edges = UniformTimedPlan.from_edges
calls = [0]


def build(*args, **kwargs):
    calls[0] += 1
    return wrong if calls[0] == 2 else real_from_edges(*args, **kwargs)


with patch.object(UniformTimedPlan, "from_edges", side_effect=build):
    injected = p.plan_uniform(req)
row = {
    "case": "final_plan_request_binding_fault_injection",
    "build_calls": calls[0],
    "requested_start": [10.0, 10.0, 0.0],
    "requested_goal": [13.0, 10.0, 0.0],
    "injected_self_audit": wrong_self.status,
}
row.update(result_fields(injected))
rows.append(row)

# The same unrelated plan under the original request's bound audit must fail.
bound = UniformAuditRequest(
    plan_id=req.plan_id, formation_id=req.formation_id, model=req.model,
    vehicle_shapes=req.vehicle_shapes, start_qf=req.start_qf,
    start_headings=req.start_headings, goal_qf=req.goal_qf,
    goal_headings=dict(wrong.goal_headings),
    timing_limits=req.successor_config.timing_limits,
    allow_reverse=req.successor_config.allow_reverse)
bound_audit = audit_uniform_timed_plan(
    bound, wrong, wrong.sample(), req.grid)
rows[-1]["bound_request_audit"] = {
    "status": bound_audit.status,
    "reason": bound_audit.reason,
    "detail": bound_audit.detail,
}
rows[-1]["injected_plan_is_self_feasible"] = (
    wrong_self.status == FEASIBLE)

# A same-endpoint replacement with a different path must also be rejected.
single = UniformTimedPlan.from_edges(
    (a_straight_edge(
        req.model, t.FormationPose(10, 10, 0), req.start_headings, (3, 0),
        {vehicle_id: 1 for vehicle_id in t.VEHICLE_IDS},
        allow_reverse=True),),
    req.vehicle_shapes, plan_id=req.plan_id,
    formation_id=req.formation_id, model=req.model)
calls = [0]


def build_single(*args, **kwargs):
    calls[0] += 1
    return single if calls[0] == 2 else real_from_edges(*args, **kwargs)


with patch.object(UniformTimedPlan, "from_edges", side_effect=build_single):
    replaced_path = p.plan_uniform(req)
row = {
    "case": "same_endpoints_different_path",
    "build_calls": calls[0],
    "replacement_self_audit": audit_uniform_timed_plan(
        UniformAuditRequest.from_plan(single), single, single.sample(),
        req.grid).status,
}
row.update(result_fields(replaced_path))
rows.append(row)

# A real final-audit refusal must still suppress the executable certificate.
real_audit = p.audit_uniform_timed_plan
audit_calls = [0]
refusal = audit_uniform_timed_plan(
    UniformAuditRequest(
        plan_id=req.plan_id, formation_id=req.formation_id, model=req.model,
        vehicle_shapes=req.vehicle_shapes, start_qf=req.start_qf,
        start_headings=req.start_headings, goal_qf=req.goal_qf,
        goal_headings=dict(wrong.goal_headings),
        timing_limits=req.successor_config.timing_limits,
        allow_reverse=req.successor_config.allow_reverse),
    wrong, wrong.sample(), req.grid)


def reject_final(*args, **kwargs):
    audit_calls[0] += 1
    if audit_calls[0] == 2:
        return refusal
    return real_audit(*args, **kwargs)


with patch.object(p, "audit_uniform_timed_plan", side_effect=reject_final):
    refused = p.plan_uniform(req)
row = {"case": "final_audit_refusal", "audit_calls": audit_calls[0]}
row.update(result_fields(refused))
rows.append(row)

# A post-failure new request produces only its own IDs/model; no reused plan.
ok = p.plan_uniform(t.make_request(t.FormationPose(11, 10, 0), plan_id=402))
row = {"case": "new_request_after_failure"}
row.update(result_fields(ok))
rows.append(row)

(OUT / "adapted_independent_probes.json").write_text(
    json.dumps(rows, indent=2) + "\n")
print(json.dumps(rows, indent=2))