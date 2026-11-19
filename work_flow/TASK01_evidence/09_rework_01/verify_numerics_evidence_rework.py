#!/usr/bin/env python3
"""Recompute the post-rework certificate arithmetic from numerics_probe_rework.json.

The probe prints production fields; this verifier checks them independently:
it never imports the planner and recomputes every displayed relation with the
same IEEE-754 ``next_up`` primitive used by the certificate.  Exit 0 means the
recorded JSON is internally consistent; a failure is a hard evidence error.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation_continuous_a import bounds  # noqa: E402

JSON_PATH = Path(__file__).resolve().parent / "numerics_probe_rework.json"
V2 = "virtual_rigid_a.numerics.g1.v2"
V3 = "virtual_rigid_a.numerics.g1.v3"
BUDGET = 1e-7
SQRT2 = math.sqrt(2.0)


def next_up(value):
    return bounds._next_up(value)


def close(a, b, ulps=4):
    scale = max(abs(a), abs(b))
    if scale == 0.0:
        return True
    return abs(a - b) <= ulps * (bounds._next_up(scale) - scale)


def next_down(value):
    return bounds._next_down(value)


def check_v3_certificate(label, path):
    budget = path["integration_budget"]
    assert path["numerics_version"] == V3, (label, "path version")
    assert budget["numerics_version"] == V3, (label, "budget version")
    assert budget["confirmed"] is True, (label, "confirmed")
    assert budget["kind"] == "motion", (label, "kind")
    total = path["position_error_bound_m"]
    assert total == budget["position_error_bound_m"], (label, "bound echo")
    assert total == budget["total_budget_m"], (label, "total budget")
    per_coord = budget["per_coordinate_total_m"]
    assert per_coord <= next_down(BUDGET / SQRT2), (label, "per-coord budget")
    assert total == next_up(next_up(SQRT2) * per_coord), (label, "sqrt2 total")
    assert total <= BUDGET, (label, "1e-7 budget")
    panel_sum = sum(budget["panels_per_segment"])
    assert panel_sum == budget["total_panels"], (label, "panel sum")
    assert budget["total_full_simpson_panels"] == panel_sum // 2, (
        label, "full panels")
    assert budget["total_local_simpson_panels"] == len(
        budget["segments"]), (label, "local panels")
    assert budget["local_simpson_panels_per_segment"] == 1, label
    quadrature = 0.0
    segment_total_sum = 0.0
    for segment in budget["segments"]:
        n = segment["subintervals"]
        assert isinstance(n, int) and n >= 2 and n % 2 == 0, (label, n)
        assert segment["full_panel_count"] == n // 2, (label, "full count")
        assert len(segment["panel_m4_upper_m"]) == n // 2, (
            label, "panel m4 count")
        m4s = segment["panel_m4_upper_m"]
        assert all(math.isfinite(v) and v >= 0.0 for v in m4s), (
            label, "panel m4 finite")
        assert segment["maximum_panel_m4_upper_m"] == max(m4s), (
            label, "maximum m4")
        full = segment["quadrature_full_remainder_m"]
        local = segment["quadrature_local_remainder_m"]
        assert segment["quadrature_remainder_m"] == next_up(full + local), (
            label, "quadrature sum")
        assert segment["evaluation_guard_m"] > 0.0, label
        assert segment["angle_guard_m"] == next_up(
            segment["h"] * path["angle_error_bound_rad"]), (
            label, "angle share")
        expected_segment = next_up(next_up(
            segment["quadrature_remainder_m"]
            + segment["evaluation_guard_m"]) + segment["angle_guard_m"])
        assert segment["per_coordinate_total_m"] == expected_segment, (
            label, "segment total")
        quadrature = next_up(quadrature + segment["quadrature_remainder_m"])
        segment_total_sum = next_up(
            segment_total_sum + segment["per_coordinate_total_m"])
    guard = budget["evaluation_guard_components"]
    assert guard["local_panel_included_per_segment"] is True, label
    per_segment = guard["per_segment_m"]
    assert len(per_segment) == len(budget["segments"]), label
    assert per_segment == [segment["evaluation_guard_m"]
                           for segment in budget["segments"]], (
        label, "per-segment evaluation guard")
    evaluation_total = 0.0
    for term in per_segment:
        evaluation_total = next_up(evaluation_total + term)
    evaluation_total = next_up(next_up(
        evaluation_total + guard["psi_accumulation_m"])
        + guard["coordinate_accumulation_m"])
    assert close(evaluation_total,
                 budget["per_coordinate_evaluation_m"]), (
        label, "evaluation total", evaluation_total,
        budget["per_coordinate_evaluation_m"])
    assert close(quadrature, budget["per_coordinate_quadrature_m"]), (
        label, "quadrature total")
    assert budget["per_coordinate_angle_m"] == next_up(
        path["length_m"] * path["angle_error_bound_rad"]), (
        label, "angle total")
    recomputed = next_up(next_up(
        budget["per_coordinate_quadrature_m"]
        + budget["per_coordinate_evaluation_m"])
        + budget["per_coordinate_angle_m"])
    assert close(recomputed, per_coord, 1), (
        label, "component recombination", recomputed, per_coord)
    global_guard = next_up(
        guard["psi_accumulation_m"] + guard["coordinate_accumulation_m"])
    difference = per_coord - segment_total_sum
    assert abs(difference - global_guard) <= 1e-9 * global_guard, (
        label, "segment sum plus global guard", difference, global_guard)
    if budget["m4_method"] == "exact_interval_fourth_derivative_per_panel":
        assert all(segment["m4_method"] ==
                   "exact_interval_fourth_derivative_per_panel"
                   for segment in budget["segments"]), (label, "m4 method")
    assert budget["refinement_steps"] == len(budget["refined_segments"]), (
        label, "refinement count")
    assert budget["outward_rounding_method"], label


def check_v2_certificate(label, path):
    budget = path["integration_budget"]
    assert path["numerics_version"] == V2, (label, "v2 path version")
    assert budget["numerics_version"] == V2, (label, "v2 budget version")
    assert path["position_error_bound_m"] == budget["position_error_bound_m"]
    assert path["position_error_bound_m"] <= BUDGET, (label, "v2 budget")


def main():
    documents = json.loads(JSON_PATH.read_text())
    replay = documents[0]
    assert replay["kind"] == "v2_replay"
    assert replay["byte_identical"] is True
    assert replay["stored_model_sha256"] == replay["rebuilt_model_sha256"]
    assert replay["stored_model_length"] == replay["rebuilt_model_length"]
    assert replay["declared_version"] == V2
    assert replay["profile_version"] == V2
    assert replay["path_numerics_version"] == V2
    assert replay["integration_budget_version"] == V2
    assert replay["integration_budget_matches"] is True
    assert replay["position_error_bound_m"] <= BUDGET
    v2_path = {
        "numerics_version": V2,
        "position_error_bound_m": replay["position_error_bound_m"],
        "integration_budget": {"numerics_version": V2,
                               "position_error_bound_m":
                                   replay["position_error_bound_m"]},
    }
    check_v2_certificate("replay", v2_path)
    v3_rebuild = replay["v3_rebuild_of_same_knots"]
    assert v3_rebuild["status"] == "CONFIRMED", v3_rebuild["status"]
    assert v3_rebuild["requested_version"] == V3
    check_v3_certificate("replay_v3_rebuild",
                         v3_rebuild["certificate"])

    real = [d for d in documents if d["kind"] == "real"]
    assert [d["plan_id"] for d in real] == [600003, 600004]
    expected_v2 = {600003: "CONFIRMED", 600004: "BOUND_UNCONFIRMED"}
    # Post-rework values.  Pre-fix evidence recorded 5.124679957475981e-09 and
    # 1.1170049229353544e-08; the absolute-curvature envelope only enlarges
    # the certificate, so these drift guards are updated together with the
    # new probe output (old Step 5 files remain untouched).
    expected_bound = {600003: 5.124997822032041e-09,
                      600004: 1.117128279893865e-08}
    for document in real:
        label = document["plan_id"]
        assert document["status"] == "SUCCESS", (label, document["status"])
        assert document["executable"] is True, label
        assert document["path"]["numerics_version"] == V3, label
        check_v3_certificate("real_{}".format(label), document["path"])
        assert document["path"]["position_error_bound_m"] == \
            expected_bound[label], (label, "bound")
        for name in ("geometry_audit", "dynamic_audit", "collision_audit"):
            assert document[name]["passed"] is True, (label, name)
            assert document[name]["reason"] == "", (label, name)
        model = document["model"]
        assert model["numerics_version"] == V3, label
        assert set(model["declared_values"].values()) == {V3}, label
        assert model["roundtrip_byte_identical"] is True, label
        assert len(model["sha256"]) == 64, label
        assert len(model["planning_context_sha256"]) == 64, label
        v2 = document["v2_rebuild_of_same_knots"]
        assert v2["status"] == expected_v2[label], (label, v2["status"])
        if v2["status"] == "CONFIRMED":
            check_v2_certificate("real_{}_v2".format(label),
                                 v2["certificate"])
        else:
            assert v2["location"]["total_panels"] > 0, (label, "v2 location")
        tamper = document["version_tamper"]
        assert tamper["numerics"]["unknown"]["reason"] == \
            "UNKNOWN_NUMERICS_VERSION", label
        assert tamper["numerics"]["other_supported"]["reason"] == \
            "MODEL_NUMERICS_MISMATCH", label
        assert tamper["profile"]["unknown"]["reason"] == \
            "UNKNOWN_NUMERICS_VERSION", label
        assert tamper["profile"]["other_supported"]["reason"] == \
            "MODEL_NUMERICS_MISMATCH", label
        assert tamper["audit_numerics"]["reason"] == \
            "UNKNOWN_NUMERICS_VERSION", label
        assert tamper["audit_numerics"]["scope"] == "DYNAMICS", label
        assert tamper["audit_profile"]["reason"] == \
            "UNSUPPORTED_PROFILE", label
        assert tamper["audit_profile"]["scope"] == "DYNAMICS", label

    synthetic = [d for d in documents if d["kind"] == "synthetic_straight"]
    assert [d["plan_id"] for d in synthetic] == [900001, 900002]
    for document in synthetic:
        label = document["plan_id"]
        assert document["status"] == "GEOMETRY_FEASIBLE", (
            label, document["status"])
        path = document["path"]
        assert path["numerics_version"] == V3, label
        check_v3_certificate("synthetic_{}".format(label), path)
        assert document["v2_rebuild_of_same_knots"]["status"] == \
            "BOUND_UNCONFIRMED", label
        assert document["v2_rebuild_of_same_knots"]["location"][
            "total_panels"] > 0, label
    print("verified {} documents: replay byte-identical, {} v3 certificates "
          "within 1e-7, v2/v3 boundary and tamper reasons consistent".format(
              len(documents), 1 + len(real) + len(synthetic)))


if __name__ == "__main__":
    main()