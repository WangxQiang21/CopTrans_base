#!/usr/bin/env python3
"""Run negative controls for the strengthened R1/R2 assertions.

The checks are in-memory only: a deliberately wrong quintic or mutated state
must make the corresponding test fail.  The delivered source/test files are
never modified.  Outputs are written below this directory, so the evidence
manifest can hash them.
"""
import dataclasses
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import unittest


def find_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "catkin_ws").is_dir() and (parent / "tests").is_dir():
            return parent
    raise RuntimeError("worktree root not found from {}".format(here))


ROOT = find_root()
HERE = Path(__file__).resolve().parent
PACKAGE_SRC = ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(PACKAGE_SRC))

import fwis_v3_optimizer.uniform_trajectory as trajectory_module
import test_uniform_trajectory as fixtures


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wheel_positions(plan):
    return tuple(
        (x, y)
        for x in (plan.limits.wheelbase / 2.0,
                  -plan.limits.wheelbase / 2.0)
        for y in (plan.limits.track / 2.0,
                  -plan.limits.track / 2.0)
    )


def mutate_state(plan, state, mutation):
    vehicles = []
    for vehicle in state.vehicles:
        if mutation == "yaw_sign_flip":
            vehicle = dataclasses.replace(
                vehicle,
                yaw_rate_radps=-vehicle.yaw_rate_radps,
                wheel_rolling_speeds=tuple(
                    -value for value in vehicle.wheel_rolling_speeds))
        elif mutation == "raw_angles_vehicle_1" and vehicle.vehicle_id == 1:
            omega = vehicle.yaw_rate_radps
            rolling = tuple(
                omega * math.hypot(x, y) for x, y in wheel_positions(plan))
            vehicle = dataclasses.replace(
                vehicle,
                wheel_angles=tuple(
                    math.atan2(x, -y) for x, y in wheel_positions(plan)),
                wheel_rolling_speeds=rolling,
                wheel_speed_mps=max(abs(value) for value in rolling))
        elif mutation == "rolling_sign_flip":
            vehicle = dataclasses.replace(
                vehicle,
                wheel_rolling_speeds=tuple(
                    -value for value in vehicle.wheel_rolling_speeds))
        elif (mutation == "two_wheels_vehicle_1"
              and vehicle.vehicle_id == 1):
            vehicle = dataclasses.replace(
                vehicle,
                wheel_rolling_speeds=vehicle.wheel_rolling_speeds[:2])
        vehicles.append(vehicle)
    return dataclasses.replace(state, vehicles=tuple(vehicles))


def run_one(case):
    method = fixtures.UniformTrajectoryAcceptance(case["method"])
    suite = unittest.TestSuite([method])
    stream = io.StringIO()
    patched = None
    saved_state_at = None
    original_quintic = trajectory_module._stable_quintic
    try:
        if case.get("clamp"):
            def clamped(tau):
                if tau <= 1e-3:
                    return original_quintic(0.0)
                if tau >= 1.0 - 1e-3:
                    return original_quintic(1.0)
                return original_quintic(tau)
            patched = clamped
            trajectory_module._stable_quintic = clamped
        mutation = case.get("mutation", "none")
        if mutation != "none":
            saved_state_at = fixtures.UniformTimedPlan.state_at

            def state_at(self, time_s, *args, **kwargs):
                return mutate_state(
                    self,
                    saved_state_at(self, time_s, *args, **kwargs),
                    mutation)

            fixtures.UniformTimedPlan.state_at = state_at
        result = unittest.TextTestRunner(
            stream=stream, verbosity=0).run(suite)
        success = result.wasSuccessful()
        detail = stream.getvalue().strip().splitlines()
        failure = None
        if result.failures:
            failure = result.failures[0][1].strip().splitlines()[-1]
        elif result.errors:
            failure = result.errors[0][1].strip().splitlines()[-1]
        return {
            "name": case["name"],
            "method": case["method"],
            "expected_test": case["expected"],
            "observed_test": "PASSED" if success else "FAILED",
            "matched_expectation":
                success == (case["expected"] == "PASSED"),
            "failure": failure,
            "output_tail": detail[-6:],
        }
    finally:
        trajectory_module._stable_quintic = original_quintic
        if saved_state_at is not None:
            fixtures.UniformTimedPlan.state_at = saved_state_at


CASES = (
    {"name": "r1_baseline", "method":
     "test_R1_stable_near_endpoint_progress_and_roundtrip",
     "expected": "PASSED"},
    {"name": "r1_large_epsilon_clamp_negative_control", "method":
     "test_R1_stable_near_endpoint_progress_and_roundtrip",
     "expected": "FAILED", "clamp": True},
    {"name": "r2_baseline", "method":
     "test_R2_signed_spin_wheel_rolling_vectors",
     "expected": "PASSED"},
    {"name": "r2_rolling_sign_flip_negative_control", "method":
     "test_R2_signed_spin_wheel_rolling_vectors",
     "expected": "FAILED", "mutation": "rolling_sign_flip"},
    {"name": "r2_yaw_sign_flip_negative_control", "method":
     "test_R2_signed_spin_wheel_rolling_vectors",
     "expected": "FAILED", "mutation": "yaw_sign_flip"},
    {"name": "r2_unfolded_angles_negative_control", "method":
     "test_R2_signed_spin_wheel_rolling_vectors",
     "expected": "FAILED", "mutation": "raw_angles_vehicle_1"},
    {"name": "r2_two_wheel_negative_control", "method":
     "test_R2_signed_spin_wheel_rolling_vectors",
     "expected": "FAILED", "mutation": "two_wheels_vehicle_1"},
)


def main():
    records = [run_one(case) for case in CASES]
    payload = {
        "schema": "fr02a.rework.mutation_controls.v1",
        "source_sha256": sha256(
            PACKAGE_SRC / "fwis_v3_optimizer" / "uniform_trajectory.py"),
        "test_sha256": sha256(ROOT / "tests" / "test_uniform_trajectory.py"),
        "controls": records,
        "all_controls_matched": all(
            item["matched_expectation"] for item in records),
        "interpretation": (
            "A negative control is successful when the deliberately wrong "
            "evaluator or mutated state makes the test FAILED; the baseline "
            "must remain PASSED."),
    }
    (HERE / "mutation_controls.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    for item in records:
        print("{}: expected {} observed {} -> {}".format(
            item["name"], item["expected_test"], item["observed_test"],
            "OK" if item["matched_expectation"] else "MISMATCH"))
        if item["failure"]:
            print("  {}".format(item["failure"]))
    return 0 if payload["all_controls_matched"] else 1


if __name__ == "__main__":
    sys.exit(main())