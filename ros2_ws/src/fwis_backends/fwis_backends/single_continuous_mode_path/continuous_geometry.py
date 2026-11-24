"""Arc-length geometry for continuous Ackermann and oblique motion blocks.

This module deliberately keeps geometry independent of timing and search. A
control knot is not a stop point. Translation blocks interpolate curvature or
oblique angle with a C1 cubic Hermite profile and integrate the resulting
vehicle-center equations directly.
"""
from dataclasses import dataclass
from functools import cached_property
import math
import time

import numpy as np

from .core import Action, Pose, advance, collapse_exact_runs


@dataclass(frozen=True)
class ControlKnot:
    s: float
    value: float
    derivative: float


@dataclass(frozen=True)
class MotionBlock:
    mode: int
    direction: int
    length: float
    knots: tuple
    # Spin uses signed_angle and has no translation length.
    signed_angle: float = 0.0
    slope_policy: str = "fixed"

    def __post_init__(self):
        if self.slope_policy not in ("fixed", "monotone"):
            raise ValueError("continuous_unknown_slope_policy")
        if self.mode not in (0, 1, 2):
            raise ValueError("continuous_unknown_mode")
        if self.direction not in (-1, 1):
            raise ValueError("continuous_invalid_direction")
        if not math.isfinite(self.length) or self.length < 0:
            raise ValueError("continuous_invalid_length")
        if not math.isfinite(self.signed_angle):
            raise ValueError("continuous_invalid_spin_angle")
        if self.mode == 2:
            if self.length != 0 or self.knots:
                raise ValueError("continuous_spin_has_translation_knots")
        else:
            if len(self.knots) < 2:
                raise ValueError("continuous_translation_needs_two_knots")
            if abs(self.knots[0].s) > 1e-10 or abs(self.knots[-1].s-self.length) > 1e-8:
                raise ValueError("continuous_knot_domain")
            if any(b.s <= a.s for a, b in zip(self.knots, self.knots[1:])):
                raise ValueError("continuous_knot_order")
            if any(not all(math.isfinite(x) for x in (k.s, k.value, k.derivative))
                   for k in self.knots):
                raise ValueError("continuous_nonfinite_knot")

    @cached_property
    def _knot_arrays(self):
        return tuple(np.asarray([getattr(k, name) for k in self.knots])
                     for name in ("s", "value", "derivative"))

    def control_many(self, stations):
        """Evaluate Hermite control and derivatives at an array of stations."""
        if self.mode == 2:
            raise ValueError("spin_has_no_translation_control")
        xs, ys, slopes = self._knot_arrays
        s = np.clip(np.asarray(stations, dtype=float), 0., self.length)
        i = np.clip(np.searchsorted(xs, s, side="right")-1, 0, len(xs)-2)
        h = xs[i+1]-xs[i]
        u = (s-xs[i])/h
        a, b, da, db = ys[i], ys[i+1], slopes[i], slopes[i+1]
        value = ((2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*h*da+
                 (-2*u**3+3*u**2)*b+(u**3-u**2)*h*db)
        first = ((6*u*u-6*u)*a+(3*u*u-4*u+1)*h*da+
                 (-6*u*u+6*u)*b+(3*u*u-2*u)*h*db)/h
        second = ((12*u-6)*a+(6*u-4)*h*da+
                  (-12*u+6)*b+(6*u-2)*h*db)/(h*h)
        return value, first, second

    @cached_property
    def _integral_prefix(self):
        xs, ys, slopes = self._knot_arrays
        h = np.diff(xs)
        return np.r_[0., np.cumsum(h*(ys[:-1]+ys[1:])/2.+
                                  h*h*(slopes[:-1]-slopes[1:])/12.)]

    def control_integral(self, stations):
        """Exact integral of the cubic control from zero to each station."""
        xs, ys, slopes = self._knot_arrays
        s = np.clip(np.asarray(stations, dtype=float), 0., self.length)
        i = np.clip(np.searchsorted(xs, s, side="right")-1, 0, len(xs)-2)
        h = xs[i+1]-xs[i]
        u = (s-xs[i])/h
        return self._integral_prefix[i]+h*(
            (.5*u**4-u**3+u)*ys[i]+(.25*u**4-2*u**3/3.+.5*u*u)*h*slopes[i]+
            (-.5*u**4+u**3)*ys[i+1]+(.25*u**4-u**3/3.)*h*slopes[i+1])

    def control(self, s):
        """Return (control, d(control)/ds, d2(control)/ds2)."""
        if self.mode == 2:
            raise ValueError("spin_has_no_translation_control")
        s = min(self.length, max(0., float(s)))
        idx = min(len(self.knots)-2,
                  max(0, int(np.searchsorted(self._knot_arrays[0], s,
                                             side="right")-1)))
        left, right = self.knots[idx], self.knots[idx+1]
        h = right.s-left.s
        u = (s-left.s)/h
        # Cubic Hermite basis and its first two physical arc-length derivatives.
        h00, h10 = 2*u**3-3*u**2+1, u**3-2*u**2+u
        h01, h11 = -2*u**3+3*u**2, u**3-u**2
        value = h00*left.value+h10*h*left.derivative+h01*right.value+h11*h*right.derivative
        ddu = ((6*u*u-6*u)*left.value + (3*u*u-4*u+1)*h*left.derivative +
               (-6*u*u+6*u)*right.value + (3*u*u-2*u)*h*right.derivative)
        d2du2 = ((12*u-6)*left.value + (6*u-4)*h*left.derivative +
                 (-12*u+6)*right.value + (6*u-2)*h*right.derivative)
        return value, ddu/h, d2du2/(h*h)


@dataclass(frozen=True)
class ContinuousPlan:
    start: Pose
    blocks: tuple
    goal: Pose
    representation: str = "continuous_motion_v2"

    def __post_init__(self):
        if self.representation != "continuous_motion_v2":
            raise ValueError("continuous_unknown_representation")


def _monotone_slopes(xs, ys):
    """Fritsch-Carlson slopes: shape preserving and C1 at every knot."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    if len(xs) < 2 or np.any(np.diff(xs) <= 0):
        raise ValueError("continuous_invalid_control_knots")
    secants = np.diff(ys)/np.diff(xs)
    if len(xs) == 2:
        # A linear profile has a continuous derivative up to both block ends.
        return np.asarray([secants[0], secants[0]])
    slopes = np.zeros_like(ys)
    for i in range(1, len(xs)-1):
        a, b = secants[i-1], secants[i]
        if a*b > 0:
            h0, h1 = xs[i]-xs[i-1], xs[i+1]-xs[i]
            w1, w2 = 2*h1+h0, h1+2*h0
            slopes[i] = (w1+w2)/(w1/a+w2/b)
    slopes[0] = ((2*(xs[1]-xs[0])+(xs[2]-xs[1]))*secants[0] -
                 (xs[1]-xs[0])*secants[1])/(xs[2]-xs[0])
    slopes[-1] = ((2*(xs[-1]-xs[-2])+(xs[-2]-xs[-3]))*secants[-1] -
                  (xs[-1]-xs[-2])*secants[-2])/(xs[-1]-xs[-3])
    for i, d in ((0, secants[0]), (-1, secants[-1])):
        if slopes[i]*d <= 0:
            slopes[i] = 0.
        elif abs(slopes[i]) > 3*abs(d):
            slopes[i] = 3*d
    return slopes



def _monotone_slopes_batch(xs, ys):
    """The same shape-preserving slopes for a bounded batch of control rows."""
    h = np.diff(xs)
    secants = np.diff(ys, axis=1)/h
    if len(xs) == 2:
        return np.repeat(secants, 2, axis=1)
    slopes = np.zeros_like(ys)
    before, after = secants[:, :-1], secants[:, 1:]
    w1, w2 = 2*h[1:]+h[:-1], h[1:]+2*h[:-1]
    with np.errstate(divide="ignore", invalid="ignore"):
        denominator = w1/before+w2/after
        np.divide(w1+w2, denominator, out=slopes[:, 1:-1], where=before*after>0)
    slopes[:, 0] = ((2*h[0]+h[1])*secants[:, 0]-h[0]*secants[:, 1])/(h[0]+h[1])
    slopes[:, -1] = ((2*h[-1]+h[-2])*secants[:, -1]-h[-1]*secants[:, -2])/(h[-1]+h[-2])
    for i, d in ((0, secants[:, 0]), (-1, secants[:, -1])):
        slopes[:, i] = np.where(slopes[:, i]*d<=0, 0.,
            np.where(np.abs(slopes[:, i])>3*np.abs(d), 3*d, slopes[:, i]))
    return slopes

def _translation_block(mode, direction, actions, transition_length=.24):
    """Build a C1 profile that stays at each primitive control between switches.

    A seed action describes a constant control over its whole arc length.  The
    old construction placed an averaged value at every action boundary, which
    made each spline interval interpolate between *two* averages.  As a
    result, control drifted across almost the entire route and the integrated
    path could be metres away from the seed.  Here each switch gets a local
    transition, bounded by the adjacent action lengths; the rest remains a
    constant-control plateau.
    """
    if not math.isfinite(transition_length) or transition_length <= 0:
        raise ValueError("continuous_invalid_transition_length")
    positions, values, total = [0.], [], 0.
    for action in actions:
        total += abs(action.amount)
        positions.append(total)
        values.append(action.control)
    # A zero derivative at the ends of each transition yields the cubic
    # smoothstep profile.  Limit every transition to less than half of either
    # adjacent primitive so transitions cannot overlap on short actions.
    knots = [(0., values[0], 0.)]
    for i, (old, new) in enumerate(zip(values, values[1:])):
        boundary = positions[i+1]
        if abs(new-old) <= 1e-12:
            continue
        left_length = positions[i+1]-positions[i]
        right_length = positions[i+2]-positions[i+1]
        width = min(transition_length, .45*left_length, .45*right_length)
        if width <= 1e-10:
            continue
        knots.append((boundary-width/2, old, 0.))
        knots.append((boundary+width/2, new, 0.))
    knots.append((total, values[-1], 0.))
    # Preserve strictly increasing knot positions; coincident controls are
    # harmless, but a numerical duplicate would make the Hermite interval
    # singular. This also handles sub-micron seed fragments defensively.
    xs, ys, ds = [], [], []
    for s, value, derivative in knots:
        if xs and s-xs[-1] <= 1e-10:
            xs[-1], ys[-1], ds[-1] = s, value, derivative
        else:
            xs.append(s)
            ys.append(value)
            ds.append(derivative)
    if len(xs) < 2:
        raise ValueError("continuous_zero_length_translation")
    knots = tuple(ControlKnot(float(s), float(v), float(d))
                  for s, v, d in zip(xs, ys, ds))
    return MotionBlock(mode, direction, float(total), knots)


def blocks_from_actions(actions, transition_length=.24):
    """Convert constant-control seeds into continuous mode/direction blocks.

    Adjacent seed primitives of the same mode and translation direction share
    one continuous control profile. Direction cusps split geometry but do not
    imply a stationary dwell. Mode changes remain explicit block boundaries.
    """
    actions = collapse_exact_runs(list(actions))
    groups, current = [], []
    current_key = None
    for action in actions:
        if abs(action.amount) <= 1e-10:
            continue
        direction = 1 if action.amount >= 0 else -1
        key = (action.mode, direction)
        if current and key != current_key:
            groups.append((current_key, current))
            current = []
        current_key = key
        current.append(action)
    if current:
        groups.append((current_key, current))
    blocks = []
    for (mode, direction), group in groups:
        if mode == 2:
            blocks.append(MotionBlock(2, direction, 0., (),
                                      signed_angle=sum(a.amount for a in group)))
        else:
            blocks.append(_translation_block(mode, direction, group, transition_length))
    return tuple(blocks)


def _derivative(block, pose, s):
    if block.mode == 0:
        kappa, _, _ = block.control(s)
        return np.asarray([block.direction*math.cos(pose[2]),
                           block.direction*math.sin(pose[2]),
                           block.direction*kappa])
    phi, _, _ = block.control(s)
    return np.asarray([block.direction*math.cos(pose[2]+phi),
                       block.direction*math.sin(pose[2]+phi), 0.])


def _rk4_step(block, pose, s, h):
    y = np.asarray(pose, dtype=float)
    k1 = _derivative(block, y, s)
    k2 = _derivative(block, y+.5*h*k1, s+.5*h)
    k3 = _derivative(block, y+.5*h*k2, s+.5*h)
    k4 = _derivative(block, y+h*k3, s+h)
    return y+(h/6.)*(k1+2*k2+2*k3+k4)


def sample_block(block, start, spacing=.01):
    """Integrate a block and return (local_s, Pose, control, mode, direction)."""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("continuous_invalid_spacing")
    out = [(0., start, 0. if block.mode == 2 else block.control(0.)[0],
            block.mode, block.direction)]
    if block.mode == 2:
        # Geometry is exact; dense angular stations keep swept-footprint
        # checks from treating a large Spin as one straight interpolation.
        count = max(1, int(math.ceil(abs(block.signed_angle)/math.radians(2))))
        for i in range(1, count+1):
            out.append((0., Pose(start.x, start.y,
                                 start.th+block.signed_angle*i/count),
                        0., 2, block.direction))
        return out
    uniform = np.linspace(0., block.length,
                          max(2, int(math.ceil(block.length/spacing))+1))
    # Split quadrature at every Hermite knot. A yaw is the exact curvature
    # integral; only x/y require quadrature. Batched four-point Gauss integration
    # avoids one Python RK4 loop per station in every endpoint-fit evaluation.
    stations = np.unique(np.r_[uniform, block._knot_arrays[0]])
    left, widths = stations[:-1], np.diff(stations)
    nodes = np.asarray([-.8611363115940526, -.3399810435848563,
                         .3399810435848563, .8611363115940526])
    weights = np.asarray([.3478548451374538, .6521451548625461,
                          .6521451548625461, .3478548451374538])
    query = left[:, None]+.5*widths[:, None]*(nodes+1.)
    if block.mode == 0:
        angle = start.th+block.direction*block.control_integral(query)
        yaw = start.th+block.direction*block.control_integral(stations)
    else:
        angle = start.th+block.control_many(query)[0]
        yaw = np.full(len(stations), start.th)
    dx = block.direction*.5*widths*np.sum(np.cos(angle)*weights, axis=1)
    dy = block.direction*.5*widths*np.sum(np.sin(angle)*weights, axis=1)
    x, y = np.r_[start.x, start.x+np.cumsum(dx)], np.r_[start.y, start.y+np.cumsum(dy)]
    controls = block.control_many(stations)[0]
    return [(float(s), Pose(float(px), float(py), float(th)), float(c),
             block.mode, block.direction)
            for s, px, py, th, c in zip(stations, x, y, yaw, controls)]


def replay_continuous(plan, spacing=.01):
    """Replay each continuous block without wrapping the internal yaw."""
    pose = plan.start
    result = [(0., pose, None, None, 0)]
    global_s = 0.
    for block in plan.blocks:
        rows = sample_block(block, pose, spacing)
        for local_s, point, control, mode, direction in rows[1:]:
            result.append((global_s+local_s, point, control, mode, direction))
        if block.mode == 2:
            pose = rows[-1][1]
        else:
            global_s += block.length
            pose = rows[-1][1]
    return result


def plan_from_actions(start, goal, actions, transition_length=.24):
    return ContinuousPlan(start, blocks_from_actions(actions, transition_length), goal)


def fit_mode_boundaries(plan, actions, deadline=None, max_iterations=40, r_min_ack=.810,
                        stop_on_feasible=False):
    """Project each mode block to its coarse boundary, ending at the true goal.

    This is a recovery initializer. Boundaries already are real mode/gear
    events; no ordinary control knot is turned into a stop.
    """
    from .core import fit_action_to_pose, wrap
    targets=[]
    pose=plan.start
    previous=None
    for action in collapse_exact_runs(actions):
        if abs(action.amount)<=1e-10:
            continue
        key=(action.mode,1 if action.amount>=0 else -1)
        if previous is not None and key!=previous:
            targets.append(pose)
        pose=advance(pose,action)
        previous=key
    targets.append(plan.goal)
    if len(targets)!=len(plan.blocks):
        raise ValueError("mode_boundary_reference_mismatch")
    fitted=[];reports=[];pose=plan.start
    for block,target in zip(plan.blocks,targets):
        if deadline is not None and time.perf_counter()>=deadline:
            raise ValueError("mode_boundary_fit_budget")
        if block.mode==2:
            if math.hypot(pose.x-target.x,pose.y-target.y)>2e-5:
                raise ValueError("mode_boundary_spin_center_mismatch")
            angle=block.signed_angle+wrap(target.th-pose.th-block.signed_angle)
            updated=MotionBlock(2,1 if angle>=0 else -1,0.,(),angle)
            report={"success":True,"kind":"analytic_spin"}
        elif block.mode==1:
            action=fit_action_to_pose(1,pose,target,.002)
            if action is None or (1 if action.amount>=0 else -1)!=block.direction:
                raise ValueError("mode_boundary_oblique_direction_or_yaw")
            length=abs(action.amount)
            updated=MotionBlock(1,block.direction,length,
                (ControlKnot(0.,action.control,0.),ControlKnot(length,action.control,0.)))
            report={"success":True,"kind":"analytic_constant_oblique"}
        else:
            local,report=fit_continuous_endpoint(
                ContinuousPlan(pose,(block,),target),r_min_ack=r_min_ack,max_iterations=max_iterations,
                terminal_xy_tol=1e-7,terminal_yaw_tol=1e-7,deadline=deadline,
                stop_on_feasible=stop_on_feasible)
            if not report["success"]:
                raise ValueError("mode_boundary_ackermann_fit_failed")
            updated=local.blocks[0]
        fitted.append(updated);reports.append(report)
        pose=sample_block(updated,pose,.025)[-1][1]
    return ContinuousPlan(plan.start,tuple(fitted),plan.goal),{
        "success":True,"projection":"coarse_mode_boundaries_true_goal",
        "blocks":reports}


def smooth_control_candidate(plan, bandwidth=.20, station_spacing=.20):
    """Regularize physical control over arc length without changing modes/gears.

    Cell averages preserve submillimetre controls' actual integral: a tiny
    terminal primitive must not become a large sampled impulse. Every result
    still requires endpoint fitting and an independent swept-body check.
    """
    from scipy.ndimage import gaussian_filter1d
    if bandwidth <= 0 or station_spacing <= 0:
        raise ValueError("continuous_invalid_smoothing_scale")
    blocks = []
    for block in plan.blocks:
        if block.mode == 2 or block.length <= .05:
            blocks.append(block)
            continue
        edges = np.linspace(0., block.length, max(4, int(math.ceil(block.length/.025))+1))
        h = edges[1]-edges[0]
        average = np.diff(block.control_integral(edges))/h
        filtered = gaussian_filter1d(average, bandwidth/h, mode="reflect")
        centers = .5*(edges[:-1]+edges[1:])
        xs = np.linspace(0., block.length,
                         max(3, int(math.ceil(block.length/station_spacing))+1))
        ys = np.interp(xs, centers, filtered)
        ds = _monotone_slopes(xs, ys)
        knots = tuple(ControlKnot(float(s), float(v), float(d))
                      for s, v, d in zip(xs, ys, ds))
        blocks.append(MotionBlock(block.mode, block.direction, block.length,
                                  knots, slope_policy="monotone"))
    return ContinuousPlan(plan.start, tuple(blocks), plan.goal)


def fit_continuous_endpoint(plan, r_min_ack=.810, integration_spacing=.025,
                            max_iterations=160, terminal_xy_tol=.015,
                            terminal_yaw_tol=math.radians(.75), deadline=None,
                            fixed_prefix_length_m=0., stop_on_feasible=False):
    """Fit block lengths and control knots to the exact terminal pose.

    The solve preserves the mode/direction block word and regularizes changes
    from the seed. It is an endpoint fitter, not a collision or dynamics
    certificate; candidates must still pass the independent evaluator.
    """
    from scipy.optimize import minimize, OptimizeResult

    layout, x0, scales, bounds = [], [], [], []
    if not math.isfinite(fixed_prefix_length_m) or fixed_prefix_length_m < 0:
        raise ValueError("continuous_invalid_fixed_prefix")
    block_starts = []
    global_s = 0.
    for block in plan.blocks:
        block_starts.append(global_s)
        global_s += block.length
    control_columns = {}
    requires_steering_fit = False
    for bi, block in enumerate(plan.blocks):
        if block.mode == 2:
            layout.append((bi, "spin", None))
            x0.append(block.signed_angle)
            scales.append(.20)
            bounds.append((block.signed_angle-1.0, block.signed_angle+1.0))
            if block_starts[bi] < fixed_prefix_length_m:
                bounds[-1] = (block.signed_angle, block.signed_angle)
            continue
        layout.append((bi, "length", None))
        x0.append(block.length)
        scales.append(max(.25, .08*block.length))
        bounds.append((max(.05, .25*block.length), max(.30, 1.8*block.length)))
        if block_starts[bi] < fixed_prefix_length_m:
            bounds[-1] = (block.length, block.length)
        for ki, knot in enumerate(block.knots):
            previous = plan.blocks[bi-1] if bi else None
            shared = (ki == 0 and previous is not None and
                      previous.mode == block.mode and previous.direction != block.direction)
            difference = previous.knots[-1].value-knot.value if shared else 0.
            if shared and block.mode == 1 and abs(abs(difference)-math.pi) < 1e-8:
                shared = False  # identical wheel axes at the +/-90 degree representation boundary
            if shared:
                column = control_columns[(bi-1, len(previous.knots)-1)]
                control_columns[(bi, ki)] = column
                if bounds[column][0] != bounds[column][1]:
                    x0[column] = .5*(x0[column]+knot.value)
                scales[column] /= math.sqrt(2.)
                requires_steering_fit |= abs(difference) > 1e-7
                continue
            control_columns[(bi, ki)] = len(layout)
            layout.append((bi, "control", ki))
            x0.append(knot.value)
            scales.append(.12 if block.mode == 0 else .20)
            if block.mode == 0:
                bounds.append((-1./r_min_ack, 1./r_min_ack))
            else:
                bounds.append((-math.pi/2, math.pi/2))
            # A changed right knot affects the preceding Hermite interval.
            interval_start = block.knots[max(0,ki-1)].s
            if block_starts[bi]+interval_start < fixed_prefix_length_m:
                bounds[-1] = (knot.value, knot.value)
    x0 = np.asarray(x0, dtype=float)
    scales = np.asarray(scales, dtype=float)
    if not len(x0):
        end = replay_continuous(plan, integration_spacing)[-1][1]
        err = np.asarray([end.x-plan.goal.x, end.y-plan.goal.y,
                          end.th-plan.goal.th])
        return plan, {"success": float(np.linalg.norm(err)) < 1e-7,
                      "status": "fixed_geometry", "terminal_error": err.tolist(),
                      "iterations": 0, "objective": 0.}

    seed_end = replay_continuous(plan, integration_spacing)[-1][1]
    goal_yaw = plan.goal.th+2*math.pi*round((seed_end.th-plan.goal.th)/(2*math.pi))
    seed_error = np.asarray([seed_end.x-plan.goal.x,
                             seed_end.y-plan.goal.y,
                             seed_end.th-goal_yaw], dtype=float)
    if (not requires_steering_fit and np.linalg.norm(seed_error[:2]) <= terminal_xy_tol and
            abs(seed_error[2]) <= terminal_yaw_tol):
        return plan, {"success": True, "status": "already_within_publish_tolerance",
                      "terminal_error": seed_error.tolist(), "iterations": 0,
                      "objective": 0.,
                      "mode_direction_word": [[b.mode, b.direction]
                                               for b in plan.blocks]}

    # Pure oblique motion preserves body yaw. A frozen A/Spin prefix followed
    # only by O motion has the same invariant. Including that constant row in
    # SLSQP's equality Jacobian makes an otherwise solvable x/y projection
    # singular. Pure Spin likewise has invariant x/y coordinates. Validate
    # those components explicitly, then constrain only movable components.
    yaw_movable = any(
        bounds[j][0] != bounds[j][1] and
        (kind == "spin" or plan.blocks[bi].mode == 0)
        for j, (bi, kind, _) in enumerate(layout))
    translation_present = any(block.mode != 2 for block in plan.blocks)
    components = ([0, 1] if translation_present else []) + ([2] if yaw_movable else [])
    invariant_xy_ok = translation_present or np.linalg.norm(seed_error[:2]) <= terminal_xy_tol
    invariant_yaw_ok = yaw_movable or abs(seed_error[2]) <= terminal_yaw_tol
    if not invariant_xy_ok or not invariant_yaw_ok:
        return plan, {"success": False, "status": "invariant_terminal_mismatch",
                      "terminal_error": seed_error.tolist(), "iterations": 0,
                      "objective": 0.,
                      "endpoint_constraint_components": [["x", "y", "yaw"][i]
                                                        for i in components]}
    if all(lower == upper for lower, upper in bounds):
        return plan, {"success": False, "status": "fixed_geometry_terminal_mismatch",
                      "terminal_error": seed_error.tolist(), "iterations": 0,
                      "objective": 0.}

    def rebuild(values):
        if deadline is not None and time.perf_counter() >= deadline:
            raise ValueError("continuous_endpoint_fit_budget")
        blocks = list(plan.blocks)
        lengths = {i: float(b.length) for i, b in enumerate(blocks)}
        controls = {i: [float(values[control_columns[(i, ki)]]) for ki in range(len(b.knots))]
                    for i, b in enumerate(blocks) if b.mode != 2}
        spins = {i: b.signed_angle for i, b in enumerate(blocks) if b.mode == 2}
        for value, (bi, kind, ki) in zip(values, layout):
            if kind == "length":
                lengths[bi] = float(value)
            elif kind == "control":
                controls[bi][ki] = float(value)
            else:
                spins[bi] = float(value)
        for bi, original in enumerate(plan.blocks):
            if original.mode == 2:
                blocks[bi] = MotionBlock(2, 1 if spins[bi] >= 0 else -1,
                                         0., (), spins[bi])
                continue
            new_length = lengths[bi]
            fractions = [k.s/original.length for k in original.knots]
            xs = [f*new_length for f in fractions]
            ys = controls[bi]
            # Fixed profiles retain the seed's local-transition geometry;
            # declared monotone profiles update slopes with fitted controls.
            derivatives = [k.derivative*original.length/new_length
                           for k in original.knots]
            preserve_slopes = block_starts[bi] < fixed_prefix_length_m
            if original.slope_policy == "monotone" and not preserve_slopes:
                # Fitted control values need fitted shape-preserving slopes.
                # Keeping a smoothed seed's old slopes can overshoot curvature
                # bounds between otherwise valid control knots.
                derivatives = _monotone_slopes(xs, ys)
            knots = tuple(ControlKnot(float(s), float(v), float(d))
                          for s, v, d in zip(xs, ys, derivatives))
            blocks[bi] = MotionBlock(original.mode, original.direction,
                                     new_length, knots, slope_policy=(
                                         "fixed" if preserve_slopes else original.slope_policy))
        return ContinuousPlan(plan.start, tuple(blocks), plan.goal)

    # Fixed normalized quadrature meshes make the residual differentiable in
    # block length. Evaluate pose sensitivities in small batches, keeping
    # memory bounded even when a seed has many control knots.
    prepared = []
    gauss_nodes = np.asarray([-.8611363115940526, -.3399810435848563,
                              .3399810435848563, .8611363115940526])
    gauss_weights = np.asarray([.3478548451374538, .6521451548625461,
                                .6521451548625461, .3478548451374538])
    for bi, block in enumerate(plan.blocks):
        columns = [(j, kind, ki) for j, (index, kind, ki) in enumerate(layout) if index == bi]
        if block.mode == 2:
            prepared.append({"block": block, "spin": columns[0][0]})
            continue
        fractions = np.asarray([k.s/block.length for k in block.knots])
        grid = np.unique(np.r_[fractions, np.linspace(0., 1.,
            max(2, int(math.ceil(block.length/integration_spacing))+1))])
        widths = np.diff(grid)
        query = (grid[:-1, None]+.5*widths[:, None]*(gauss_nodes+1.)).ravel()
        weights = (.5*widths[:, None]*gauss_weights).ravel()
        index = np.clip(np.searchsorted(fractions, query, side="right")-1, 0, len(fractions)-2)
        h = fractions[index+1]-fractions[index]
        u = (query-fractions[index])/h
        # The normalized mesh is fixed throughout SLSQP, including all
        # finite-difference batches. Its Hermite basis need not be rebuilt
        # for every residual/Jacobian evaluation.
        integral_basis = (.5*u**4-u**3+u,
                          (.25*u**4-2*u**3/3.+.5*u*u)*h,
                          -.5*u**4+u**3,
                          (.25*u**4-u**3/3.)*h)
        control_basis = (2*u**3-3*u*u+1,
                         (u**3-2*u*u+u)*h,
                         -2*u**3+3*u*u,
                         (u**3-u*u)*h)
        prepared.append({"block": block, "fractions": fractions,
            "preserve_slopes": block_starts[bi] < fixed_prefix_length_m,
            "length": next(j for j, kind, _ in columns if kind == "length"),
            "controls": [control_columns[(bi, ki)] for ki in range(len(block.knots))],
            "index": index, "h": h, "u": u, "weights": weights,
            "integral_basis": integral_basis, "control_basis": control_basis})

    last_residual_key = None
    last_residual_value = None
    residual_cache_hits = 0

    def batch_residual(values):
        nonlocal last_residual_key, last_residual_value, residual_cache_hits
        if deadline is not None and time.perf_counter() >= deadline:
            raise ValueError("continuous_endpoint_fit_budget")
        # SLSQP requests the same base residual for the constraint, its
        # Jacobian and the feasibility callback. Cache only exact one-row
        # inputs; perturbed finite-difference batches remain independent.
        cache_key = values.tobytes() if len(values) == 1 else None
        if cache_key is not None and cache_key == last_residual_key:
            residual_cache_hits += 1
            return last_residual_value.copy()
        count = len(values)
        x = np.full(count, plan.start.x, dtype=float)
        y = np.full(count, plan.start.y, dtype=float)
        yaw = np.full(count, plan.start.th, dtype=float)
        for item in prepared:
            block = item["block"]
            if block.mode == 2:
                yaw += values[:, item["spin"]]
                continue
            length = values[:, item["length"]]
            controls = values[:, item["controls"]]
            xs = item["fractions"]
            if block.slope_policy == "monotone" and not item["preserve_slopes"]:
                slopes = _monotone_slopes_batch(xs, controls)
            else:
                slopes = np.broadcast_to(np.asarray([k.derivative*block.length
                    for k in block.knots]), controls.shape)
            i, h, u = item["index"], item["h"], item["u"]
            left, right = controls[:, i], controls[:, i+1]
            dl, dr = slopes[:, i], slopes[:, i+1]
            if block.mode == 0:
                gaps = np.diff(xs)
                areas = gaps*(controls[:, :-1]+controls[:, 1:])/2.+                        gaps*gaps*(slopes[:, :-1]-slopes[:, 1:])/12.
                prefix = np.c_[np.zeros(count), np.cumsum(areas, axis=1)]
                b00,b10,b01,b11 = item["integral_basis"]
                integral = prefix[:, i]+h*(
                    b00*left+b10*dl+b01*right+b11*dr)
                angle = yaw[:, None]+block.direction*length[:, None]*integral
            else:
                b00,b10,b01,b11 = item["control_basis"]
                control = b00*left+b10*dl+b01*right+b11*dr
                angle = yaw[:, None]+control
            x += block.direction*length*np.sum(np.cos(angle)*item["weights"], axis=1)
            y += block.direction*length*np.sum(np.sin(angle)*item["weights"], axis=1)
            if block.mode == 0:
                yaw += block.direction*length*prefix[:, -1]
        result = np.column_stack((x-plan.goal.x, y-plan.goal.y, yaw-goal_yaw))
        if cache_key is not None:
            last_residual_key, last_residual_value = cache_key, result.copy()
        return result

    def residual(values):
        return batch_residual(np.asarray(values)[None, :])[0, components]

    def residual_jacobian(values):
        values = np.asarray(values)
        base = residual(values)
        upper = np.asarray([pair[1] for pair in bounds])
        step = math.sqrt(np.finfo(float).eps)*(1.+np.abs(values))
        step = np.where(values+step<=upper, step, -step)
        jacobian = np.empty((len(components), len(values)))
        for first in range(0, len(values), 16):
            indices = np.arange(first, min(len(values), first+16))
            batch = np.repeat(values[None, :], len(indices), axis=0)
            batch[np.arange(len(indices)), indices] += step[indices]
            jacobian[:, indices] = ((batch_residual(batch)[:, components]-base)/
                                    step[indices, None]).T
        return jacobian

    def objective(values):
        delta = (values-x0)/scales
        return .5*float(np.dot(delta, delta))

    class ProjectionReady(Exception):
        def __init__(self, values):
            self.values = np.asarray(values).copy()

    callback_iterations = 0
    def projection_callback(values):
        nonlocal callback_iterations
        callback_iterations += 1
        error = batch_residual(np.asarray(values)[None,:])[0]
        if np.linalg.norm(error[:2]) <= 1e-5 and abs(error[2]) <= 1e-5:
            raise ProjectionReady(values)

    stopped_on_feasible = False
    try:
        result = minimize(objective, x0, method="SLSQP", bounds=bounds,
                          jac=lambda values: (values-x0)/(scales*scales),
                          constraints=[{"type": "eq", "fun": residual,
                                        "jac": residual_jacobian}],
                          callback=projection_callback if stop_on_feasible else None,
                          options={"maxiter": int(max_iterations), "ftol": 1e-11,
                                   "disp": False})
    except ProjectionReady as ready:
        # Endpoint projection only needs feasibility; the trajectory scorer
        # and strict independent geometry audit still decide admissibility.
        stopped_on_feasible = True
        result = OptimizeResult(x=ready.values,success=False,nit=callback_iterations,
            fun=objective(ready.values),message="endpoint tolerance reached")
    fitted = rebuild(result.x)
    actual = replay_continuous(fitted, integration_spacing)[-1][1]
    terminal = np.asarray([actual.x-plan.goal.x, actual.y-plan.goal.y, actual.th-goal_yaw])
    passed = (np.linalg.norm(terminal[:2]) <= (
                  2e-5 if translation_present else terminal_xy_tol) and
              abs(terminal[2]) <= (2e-5 if yaw_movable else terminal_yaw_tol))
    return fitted, {"success": bool(passed), "solver_success": bool(result.success),
                    "stopped_on_feasible": stopped_on_feasible,
                    "solver_message": str(result.message),
                    "terminal_error": terminal.tolist(),
                    "iterations": int(result.nit),
                    "fixed_prefix_length_m": fixed_prefix_length_m,
                    "endpoint_jacobian": "batched_finite_difference_16",
                    "endpoint_residual_cache_hits": residual_cache_hits,
                    "endpoint_constraint_components": [["x", "y", "yaw"][i]
                                                        for i in components],
                    "objective": float(result.fun),
                    "mode_direction_word": [[b.mode, b.direction]
                                             for b in fitted.blocks]}


def fit_continuous_suffix(plan, tail_length_m=3., control_spacing_m=.4, **kwargs):
    """Project the goal using a bounded suffix, preserving the prefix exactly.

    Insert exact Hermite knots into the suffix so a long constant-control
    plateau can bend locally. This does not split motion blocks or introduce
    stops. Sweep and dynamics remain the caller's acceptance gates.
    """
    if not all(math.isfinite(x) and x > 0 for x in (tail_length_m, control_spacing_m)):
        raise ValueError("continuous_invalid_suffix_window")
    total = sum(b.length for b in plan.blocks)
    prefix = max(0., total-tail_length_m)
    blocks, global_s = [], 0.
    for block in plan.blocks:
        if block.mode == 2:
            blocks.append(block)
            continue
        local_start = max(0., prefix-global_s)
        if local_start < block.length:
            extra = np.linspace(local_start, block.length,
                max(2, int(math.ceil((block.length-local_start)/control_spacing_m))+1))
            stations = sorted(k.s for k in block.knots)
            for station in extra:
                if min(abs(station-s) for s in stations) > 1e-9:
                    stations.append(float(station))
            stations.sort()
            knots = tuple(ControlKnot(s,*block.control(s)[:2]) for s in stations)
            block = MotionBlock(block.mode,block.direction,block.length,knots,
                                slope_policy="fixed")
        blocks.append(block)
        global_s += block.length
    enriched = ContinuousPlan(plan.start,tuple(blocks),plan.goal)
    fitted, report = fit_continuous_endpoint(enriched,
        fixed_prefix_length_m=prefix, **kwargs)
    report.update(projection="fixed_prefix_suffix", tail_length_m=tail_length_m,
                  control_spacing_m=control_spacing_m, fixed_prefix_length_m=prefix)
    return fitted, report


def constant_action_endpoint_error(start, actions, spacing=.01):
    """Diagnostic: compare continuous geometry against the constant seed."""
    from .core import replay
    target = replay(start, actions)[-1]
    plan = ContinuousPlan(start, blocks_from_actions(actions), target)
    end = replay_continuous(plan, spacing)[-1][1]
    return math.hypot(end.x-target.x, end.y-target.y), end.th-target.th
