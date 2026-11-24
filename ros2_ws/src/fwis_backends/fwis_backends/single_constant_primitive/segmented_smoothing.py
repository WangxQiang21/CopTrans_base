"""Paper-08-inspired quadratic smoothing inside fixed A/mode segments.

The graph still decides modes and directions. A small convex quadratic problem
regularizes Ackermann curvature, then each modified segment is projected back
to its original shared boundary. Only a fully audited, better timed candidate
can replace the incumbent.
"""
import math
import time

import numpy as np

from .continuous_geometry import (ContinuousPlan, ControlKnot, MotionBlock,
                                  fit_continuous_endpoint, sample_block)
from .continuous_retiming import retime_continuous


def quadratic_smooth_segments(plan, r_min_ack=.810, weight=3., spacing=.30):
    if weight <= 0 or spacing <= 0 or r_min_ack <= 0:
        raise ValueError("segment_qp_invalid_parameters")
    blocks, records = [], []
    for index, block in enumerate(plan.blocks):
        if block.mode != 0 or block.length < .30:
            blocks.append(block)
            continue
        count = min(41, max(5, int(math.ceil(block.length/spacing))+1))
        xs = np.linspace(0., block.length, count)
        original = block.control_many(xs)[0]
        if np.ptp(original) < 1e-7:
            blocks.append(block)
            continue
        second = np.diff(np.eye(count), n=2, axis=0)
        hessian = np.eye(count)+weight*second.T@second
        # Zero Hermite slopes give an exact trapezoidal curvature integral.
        # Fix both boundary wheel settings and preserve the total yaw change.
        integral = np.full(count, block.length/(count-1))
        integral[[0, -1]] *= .5
        equalities = np.vstack((np.eye(count)[0], np.eye(count)[-1], integral))
        target = np.asarray([original[0], original[-1],
                             block.control_integral(block.length)])
        matrix = np.block([[hessian, equalities.T],
                           [equalities, np.zeros((3, 3))]])
        values = np.linalg.solve(matrix, np.r_[original, target])[:count]
        bound = 1./r_min_ack
        if np.max(np.abs(values)) > bound+1e-9:
            from scipy.optimize import Bounds, LinearConstraint, minimize
            result = minimize(lambda x: .5*x@hessian@x-original@x,
                np.clip(values, -bound, bound), jac=lambda x: hessian@x-original,
                method="SLSQP", bounds=Bounds(-bound, bound),
                constraints=[LinearConstraint(equalities, target, target)],
                options={"maxiter": 60, "ftol": 1e-10})
            if not result.success:
                blocks.append(block)
                records.append({"block_index": index, "status": "qp_rejected"})
                continue
            values = result.x
        if np.max(np.abs(equalities@values-target)) > 1e-7:
            raise ValueError("segment_qp_boundary_or_integral_residual")
        before = float(np.sum((second@original)**2))
        after = float(np.sum((second@values)**2))
        if after >= before-1e-10:
            blocks.append(block)
            continue
        knots = tuple(ControlKnot(float(s), float(c), 0.)
                      for s, c in zip(xs, values))
        blocks.append(MotionBlock(0, block.direction, block.length, knots))
        records.append({"block_index": index, "status": "smoothed",
                        "second_difference_before": before,
                        "second_difference_after": after,
                        "knots": count})
    return ContinuousPlan(plan.start, tuple(blocks), plan.goal), {
        "objective": "curvature_deviation_plus_second_difference",
        "changed_blocks": sum(r["status"] == "smoothed" for r in records),
        "blocks": records}


def improve_segmented_trajectory(plan, timed, audit, evaluator,
                                allow_reverse=False, budget_s=.8):
    """Retain the certified incumbent unless segment QP improves its ranking."""
    begin = time.perf_counter()
    deadline = begin+budget_s
    report = {"selected": False, "baseline_duration_s": float(timed.duration)}
    try:
        smoothed, detail = quadratic_smooth_segments(
            plan, evaluator.limits.r_min_ack)
        report.update(detail)
        if not detail["changed_blocks"]:
            report["status"] = "no_varying_ackermann_segment"
            return plan, timed, audit, report
        fitted, boundaries = [], []
        reference_pose = pose = plan.start
        for original, block in zip(plan.blocks, smoothed.blocks):
            target = sample_block(original, reference_pose, .025)[-1][1]
            if block is not original:
                local, fit = fit_continuous_endpoint(
                    ContinuousPlan(pose, (block,), target),
                    evaluator.limits.r_min_ack, max_iterations=60,
                    terminal_xy_tol=1e-6, terminal_yaw_tol=1e-6,
                    deadline=deadline, stop_on_feasible=True)
                if not fit.get("success"):
                    raise ValueError("segment_qp_boundary_projection_failed")
                block = local.blocks[0]
            fitted.append(block)
            pose = sample_block(block, pose, .025)[-1][1]
            boundaries.append([target.x, target.y, target.th])
            reference_pose = target
        candidate = ContinuousPlan(plan.start, tuple(fitted), plan.goal)
        if evaluator.continuous_collision(candidate) is not None:
            raise ValueError("segment_qp_sweep_rejected")
        candidate_timed = retime_continuous(candidate, evaluator.limits,
            dt=.02, spatial_dt=evaluator.profile.sweep_spacing)
        candidate_audit = evaluator.evaluate_continuous_geometry(
            candidate, candidate_timed, allow_reverse=allow_reverse)
        report.update(status="audited", candidate_duration_s=candidate_timed.duration,
                      audit_status=candidate_audit.get("status"),
                      shared_boundaries=boundaries)
        faster = candidate_timed.duration < timed.duration-1e-6
        equal_better = (abs(candidate_timed.duration-timed.duration) <= 1e-6 and
                        candidate_audit.get("score", math.inf) <
                        audit.get("score", math.inf)-1e-9)
        if candidate_audit.get("status") == "FEASIBLE" and (faster or equal_better):
            report["selected"] = True
            return candidate, candidate_timed, candidate_audit, report
        return plan, timed, audit, report
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        report.update(status="incumbent_retained", reason=str(exc))
        return plan, timed, audit, report
    finally:
        report["elapsed_s"] = time.perf_counter()-begin
