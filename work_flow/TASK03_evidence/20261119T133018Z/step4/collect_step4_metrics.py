#!/usr/bin/env python3
"""TASK03 Step 4 evidence: measured continuous F+OBB certificate behaviour.

Assertions live in ``FormationContinuousCertificateStep4Tests`` in
``tests/test_virtual_a_formation_corridor.py``.  This script records the same
measured numbers: a wide-scene VERIFIED certificate (twice, for hash
determinism), a tight wall that is collocation-safe but continuously violated,
a genuinely clear tight wall, the interval/depth/time limits, A/D
degeneration, non-finite interval data, identity/payload tamper, the f_only key
set, and all optimizer gate outcomes.

Run from the worktree root with the TASK03 plan's PYTHONPATH.
"""
import copy
import json
import math
import os
import sys
import traceback
from unittest import mock

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    optimizer as optimizer_module,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    FORMATION_CORRIDOR_SCOPE_F_ONLY,
    GeometryResult,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    allocate_corridors,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (  # noqa: E402
    build_formation_template_candidates,
)
from fwis_v3_optimizer.formation_continuous_a.formation_certificate import (  # noqa: E402
    _seal as seal_certificate,
    certify_formation_path,
    verify_formation_certificate,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (  # noqa: E402
    C2CurvatureProfile,
    VirtualAPath,
)
from tests.test_virtual_a_formation_corridor import (  # noqa: E402
    CONTEXT_SHA,
    INTERVALS,
    MARGIN_M,
    PORTAL_THETA,
    S_LENGTH,
    _assess_portal,
    _certificate_for,
    _constant_profile,
    _fake_ipopt_result,
    _portal_scene,
    _scene,
    _slanted_wall_scene,
    _z,
    solve_open_path_guided,
)
from tests.test_virtual_a_optimizer import (  # noqa: E402
    request_from_fixture,
)


def _certificate_summary(certificate):
    witness = certificate.get("witness")
    return {
        "status": certificate.get("status"),
        "reason": certificate.get("reason"),
        "schema": certificate.get("schema"),
        "is_continuous_certificate":
            certificate.get("is_continuous_certificate"),
        "finite_sample_audit_is_certificate":
            certificate.get("finite_sample_audit_is_certificate"),
        "intervals_checked": certificate.get("intervals_checked"),
        "intervals_verified": certificate.get("intervals_verified"),
        "max_depth_reached": certificate.get("max_depth_reached"),
        "max_outward_residual_m":
            certificate.get("max_outward_residual_m"),
        "unclosed_interval_count":
            certificate.get("unclosed_interval_count"),
        "unclosed_intervals": list(
            certificate.get("unclosed_intervals") or ()),
        "cut_check_count": len(certificate.get("cut_checks") or ()),
        "cut_stations": sorted({
            round(float(check["s_m"]), 9)
            for check in certificate.get("cut_checks") or ()}),
        "cut_statuses": sorted({
            str(check.get("status"))
            for check in certificate.get("cut_checks") or ()}),
        "vehicle_ids": sorted({
            int(item["vehicle_id"])
            for item in certificate.get("vehicle_coverage") or ()}),
        "f_only_audit": certificate.get("f_only_audit"),
        "witness": witness,
        "timed_out": certificate.get("timed_out"),
        "interval_limit_reached":
            certificate.get("interval_limit_reached"),
        "certificate_sha256": certificate.get("certificate_sha256"),
        "path_sha256": certificate.get("path_sha256"),
        "elapsed_s": certificate.get("elapsed_s"),
    }


def _verification_summary(result):
    return {
        "passed": bool(result.get("passed")),
        "reason": result.get("reason"),
        "detail": result.get("detail"),
        "status": result.get("status"),
        "certificate_sha256": result.get("certificate_sha256"),
    }


def _verify(scene, certificate, *, path=None, request=None, chain=None,
            template=None, context=CONTEXT_SHA):
    return verify_formation_certificate(
        certificate,
        scene["path"] if path is None else path,
        scene["request"] if request is None else request,
        scene["chain"] if chain is None else chain,
        scene["template"] if template is None else template,
        allocation=scene["allocation"], guide=scene["guide"],
        planning_context_sha256=context)


def _fake_solve(request, config, packet, guess, wall_limit_s=None,
                corridor_constraint=None, **kwargs):
    return _fake_ipopt_result(_z(0.0))


def _gate_solve(scene, **patches):
    with mock.patch.object(
            optimizer_module, "_solve_with_ipopt", side_effect=_fake_solve):
        with mock.patch.object(
                optimizer_module, "audit_geometry") as audit_mock:
            patchers = []
            try:
                for name, patch in patches.items():
                    patcher = mock.patch.object(
                        optimizer_module, name, **patch)
                    patcher.start()
                    patchers.append(patcher)
                result = solve_open_path_guided(
                    scene["request"], scene["config"],
                    route_guides=(scene["guide"],),
                    corridor_chain=scene["chain"],
                    corridor_allocation=scene["allocation"],
                    formation_template=scene["template"],
                    formation_planning_context_sha256=CONTEXT_SHA)
            finally:
                for patcher in reversed(patchers):
                    patcher.stop()
    attempt = result.attempts[-1] if result.attempts else None
    return {
        "status": result.status,
        "reason": result.reason,
        "deadline_exhausted": result.deadline_exhausted,
        "audit_called": bool(audit_mock.called),
        "attempt_status": None if attempt is None else attempt.status,
        "attempt_rejection": (
            None if attempt is None else attempt.rejection_reason),
        "certificate_status":
            (result.formation_certificate or {}).get("status"),
        "result_dict_certificate_status": (
            result.as_dict().get("formation_certificate") or {}).get(
                "status"),
        "path_is_none": result.path is None,
    }


def _measure(record, name, function):
    try:
        record[name] = function()
    except Exception as exc:  # evidence must show failures, never hide them
        record[name] = {
            "error": "{}: {}".format(type(exc).__name__, exc),
            "traceback": traceback.format_exc(),
        }


def main():
    evidence = {}

    def wide_scene():
        scene = _scene()
        first = _certificate_for(scene)
        second = _certificate_for(scene)
        return {
            "first": _certificate_summary(first),
            "second_sha256": second["certificate_sha256"],
            "deterministic":
                first["certificate_sha256"] == second["certificate_sha256"],
            "verification": _verification_summary(
                _verify(scene, first)),
        }

    def e08_scene():
        scene = _slanted_wall_scene()
        certificate = _certificate_for(scene)
        witness = certificate.get("witness") or {}
        return {
            "station_max_m": scene["station_max_m"],
            "fine_max_m": scene["fine_max_m"],
            "wall_m": scene["wall_m"],
            "gap_m": scene["fine_max_m"] - scene["station_max_m"],
            "certificate": _certificate_summary(certificate),
            "witness_fraction": (
                None if not witness else
                float(witness["s_m"]) / S_LENGTH),
            "verification": _verification_summary(
                _verify(scene, certificate)),
        }

    def clear_scene():
        scene = _slanted_wall_scene(clearance_m=1e-3)
        certificate = _certificate_for(scene)
        return {
            "station_max_m": scene["station_max_m"],
            "fine_max_m": scene["fine_max_m"],
            "wall_m": scene["wall_m"],
            "clearance_m": scene["wall_m"] - scene["fine_max_m"],
            "certificate": _certificate_summary(certificate),
            "verification": _verification_summary(
                _verify(scene, certificate)),
        }

    def limits():
        scene = _slanted_wall_scene(clearance_m=1e-3)
        return {
            "max_depth_zero": _certificate_summary(
                _certificate_for(scene, max_depth=0)),
            "max_intervals_one": _certificate_summary(
                _certificate_for(scene, max_intervals=1)),
            "max_seconds_zero": _certificate_summary(
                _certificate_for(scene, max_seconds=0.0)),
        }

    def degeneration():
        scene = _scene(kappa=1.0)
        certificate = _certificate_for(scene, max_depth=4)
        return {
            "certificate": _certificate_summary(certificate),
            "verification": _verification_summary(
                _verify(scene, certificate)),
        }

    def nonfinite():
        scene = _scene()
        real_bounds = C2CurvatureProfile.interval_bounds

        def nan_bounds(self, s0, s1):
            record = dict(real_bounds(self, s0, s1))
            record["k"] = [float("nan"), float("nan")]
            return record

        with mock.patch.object(
                C2CurvatureProfile, "interval_bounds", nan_bounds):
            certificate = _certificate_for(scene)
        return _certificate_summary(certificate)

    def tamper():
        from dataclasses import replace
        from fwis_v3_optimizer.formation import FormationPose
        from fwis_v3_optimizer.formation_continuous_a.contracts import (
            CorridorChain,
        )
        scene = _scene()
        certificate = _certificate_for(scene)
        result = {"baseline": _verification_summary(
            _verify(scene, certificate))}
        other_path = VirtualAPath(
            scene["request"].start_qf, _constant_profile(0.2),
            scene["request"].model)
        result["wrong_path"] = _verification_summary(
            _verify(scene, certificate, path=other_path))
        wrong_request = replace(
            scene["request"],
            goal_qf=FormationPose(
                scene["request"].goal_qf.x,
                scene["request"].goal_qf.y,
                scene["request"].goal_qf.psi + 0.05))
        result["wrong_request"] = _verification_summary(
            _verify(scene, certificate, request=wrong_request))
        wrong_chain = CorridorChain(
            regions=scene["chain"].regions,
            route_source=scene["chain"].route_source,
            grid_sha256="1" * 64)
        result["wrong_grid_chain"] = _verification_summary(
            _verify(scene, certificate, chain=wrong_chain))
        result["wrong_context"] = _verification_summary(
            _verify(scene, certificate, context="d" * 64))
        result["wrong_template"] = _verification_summary(
            _verify(
                scene, certificate,
                template=replace(
                    scene["template"], margin_source="tampered")))
        body = copy.deepcopy(certificate)
        body["max_outward_residual_m"] = 1.0
        result["tampered_body"] = _verification_summary(
            _verify(scene, body))
        timing = copy.deepcopy(certificate)
        timing["elapsed_s"] = float(timing["elapsed_s"]) + 1.0
        result["elapsed_only"] = _verification_summary(
            _verify(scene, timing))
        payload = copy.deepcopy(certificate)
        payload.pop("certificate_sha256")
        payload["unclosed_interval_count"] = "not-an-int"
        result["signed_bad_count"] = _verification_summary(
            _verify(scene, seal_certificate(payload)))
        return result

    def compact_scene():
        # One valid certificate for a split template proves the template cut
        # is a real two-sided station in the interval partition.
        from dataclasses import replace
        scene = _scene()
        base = scene["template"]
        left = replace(base.segments[0], sigma_end=0.4)
        right = replace(base.segments[0], sigma_start=0.4)
        split = replace(base, segments=(left, right))
        certificate = _certificate_for(scene, template=split)
        return {
            "template_id_changed": split.template_id != base.template_id,
            "certificate": _certificate_summary(certificate),
            "verification": _verification_summary(
                _verify(scene, certificate, template=split)),
        }

    def portal_transition():
        scene = _portal_scene(overlap_half=0.3)
        theta = PORTAL_THETA
        start = (-2.0 * math.cos(theta), -2.0 * math.sin(theta))
        goal = (2.0 * math.cos(theta), 2.0 * math.sin(theta))
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(start[0], start[1], theta),
            start_headings={
                vehicle_id: theta
                for vehicle_id in scene["request"].start_headings},
            goal_qf=FormationPose(goal[0], goal[1], theta))
        allocation = allocate_corridors(
            scene["chain"], scene["guide"], INTERVALS)
        assessment = _assess_portal(
            scene["chain"], scene["guide"], request, scene["grid"],
            allocation, theta, theta)
        candidates = build_formation_template_candidates(
            request, scene["chain"], scene["guide"], allocation,
            assessment, scene["config"],
            planning_context_sha256=CONTEXT_SHA, margin_m=MARGIN_M,
            margin_source="task03_step4_portal_fixture")
        template = candidates.templates[0]
        path = VirtualAPath(
            request.start_qf, _constant_profile(0.0, length=4.0),
            request.model)
        certificate = certify_formation_path(
            path, request, scene["chain"], template,
            allocation=allocation, guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA,
            config=scene["config"])
        sides_by_station = {}
        for check in certificate["cut_checks"]:
            sides_by_station.setdefault(
                round(float(check["s_m"]), 9), set()).add(check["side"])
        boundaries = [
            round(float(segment.sigma_start) * 4.0, 9)
            for segment in template.segments[1:]]
        return {
            "template_count": len(candidates.templates),
            "portal_boundaries_m": boundaries,
            "boundary_sides": {
                repr(boundary): sorted(sides_by_station.get(boundary, ()))
                for boundary in boundaries},
            "certificate": _certificate_summary(certificate),
            "verification": _verification_summary(
                verify_formation_certificate(
                    certificate, path, request, scene["chain"], template,
                    allocation=allocation, guide=scene["guide"],
                    planning_context_sha256=CONTEXT_SHA)),
        }

    def gate():
        scene = _scene()
        return {
            "unconfirmed": _gate_solve(
                scene, certify_formation_path={"return_value": {
                    "status": "UNCONFIRMED",
                    "reason": "INTERVAL_NOT_CLOSED",
                    "timed_out": False}}),
            "verified_but_failed_verify": _gate_solve(
                scene,
                certify_formation_path={"return_value": {
                    "status": "VERIFIED", "reason": "",
                    "timed_out": False}},
                verify_formation_certificate={"return_value": {
                    "passed": False,
                    "reason": "CERTIFICATE_HASH_MISMATCH"}}),
            "timeout": _gate_solve(
                scene, certify_formation_path={"return_value": {
                    "status": "UNCONFIRMED", "reason": "TIME_LIMIT",
                    "timed_out": True}}),
            "accepted_real_certificate": _gate_solve(scene),
        }

    def result_keys():
        f_only = GeometryResult(status="NO_MOTION").as_dict()
        union = GeometryResult(
            status="NO_MOTION",
            formation_corridor_scope=(
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
            formation_certificate={"status": "VERIFIED"}).as_dict()
        return {
            "f_only_scope": FORMATION_CORRIDOR_SCOPE_F_ONLY,
            "f_only_keys": sorted(f_only),
            "has_formation_certificate_in_f_only":
                "formation_certificate" in f_only,
            "union_has_formation_certificate":
                "formation_certificate" in union,
            "union_certificate_status":
                union["formation_certificate"]["status"],
        }

    for name, function in (
            ("wide_verified", wide_scene),
            ("e08_collocation_safe_continuous_violation", e08_scene),
            ("tight_clear_verified", clear_scene),
            ("limits", limits),
            ("normal_branch_degeneration", degeneration),
            ("nonfinite_interval", nonfinite),
            ("tamper", tamper),
            ("split_template_cut", compact_scene),
            ("portal_transition_cut", portal_transition),
            ("optimizer_gate", gate),
            ("result_key_sets", result_keys)):
        _measure(evidence, name, function)
    evidence["definite_margin_m"] = 1e-9
    evidence["margin_m"] = MARGIN_M
    evidence["context_sha256"] = CONTEXT_SHA
    print(json.dumps(evidence, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()