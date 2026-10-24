"""Independent direction audit of published samples, with no planner imports."""
import math


def audit_direction(samples, allow_reverse=False, report=None, allow_shunting=True):
    """samples: (time, x, y, body_yaw, signed_speed); Spin/holds have zero v.

    Check body projection against the signed velocity on each moving interval.
    Exact lateral travel has zero body projection, so its gear is established
    by the signed speed. Safety and interpolation are separate checks.
    """
    backward = forward = 0.
    mismatches = 0
    previous_gear=None
    spin_since_motion=0.
    gear_reversals=0
    reversals_without_spin=0
    for a, b in zip(samples, samples[1:]):
        _, x, y, yaw, v = a
        _, nx, ny, nyaw, nv = b
        distance = math.hypot(nx-x, ny-y)
        delta = math.atan2(math.sin(nyaw-yaw), math.cos(nyaw-yaw))
        if distance <= 1e-10:
            spin_since_motion+=delta
            continue
        heading = yaw+.5*delta
        projection = (nx-x)*math.cos(heading)+(ny-y)*math.sin(heading)
        speed = .5*(v+nv)
        gear=1 if speed>=0 else -1
        if previous_gear is not None and gear!=previous_gear:
            gear_reversals+=1
            if abs(math.atan2(math.sin(spin_since_motion),math.cos(spin_since_motion)))<=1e-4:
                reversals_without_spin+=1
        previous_gear,spin_since_motion=gear,0.
        if ((projection < -1e-7 and speed >= -1e-10) or
                (projection > 1e-7 and speed <= 1e-10) or abs(speed) <= 1e-10):
            mismatches += 1
        if speed < -1e-10:
            backward += distance
        else:
            forward += distance
    reverse = backward > 1e-8
    all_reverse = reverse and forward <= 1e-8
    # Gear policy is explicit configuration. Candidate provenance and the
    # presence of a forward alternative do not veto a valid reverse route;
    # route selection and preference belong to the shared trajectory cost.
    permitted = not reverse or bool(allow_reverse)
    no_shunting=(gear_reversals<=1 and reversals_without_spin==0)
    return {"pass_check": not mismatches and permitted and (allow_shunting or no_shunting),
            "gear_reversals":gear_reversals,
            "reversals_without_spin":reversals_without_spin,
            "shunting_allowed_by_config":bool(allow_shunting),
            "signed_direction_mismatches": mismatches,
            "forward_distance_m": forward, "reverse_distance_m": backward,
            "all_reverse_route": all_reverse,
            "reverse_allowed_by_config": bool(allow_reverse)}
