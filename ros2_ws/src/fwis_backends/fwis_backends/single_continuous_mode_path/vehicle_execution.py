"""Shared vehicle execution without changing a planner's exact geometry.

The default simulation profile bounds longitudinal speed/acceleration/jerk
and Spin dynamics. Steering jumps are diagnostic in that profile. Restricted
execution uses the existing stationary, rate-bounded primitive retimer.
"""
from dataclasses import dataclass, asdict
from functools import cached_property
import hashlib
import json
import math
import numpy as np
from .core import Pose, Action, replay, collapse_exact_runs
from .continuous_geometry import ContinuousPlan, MotionBlock, ControlKnot
from .continuous_retiming import retime_continuous


@dataclass(frozen=True)
class ExactControlBlock:
    mode: int
    direction: int
    actions: tuple
    signed_angle: float = 0.
    slope_policy: str = 'exact_piecewise_constant'

    @property
    def length(self):
        return sum(abs(a.amount) for a in self.actions)

    @cached_property
    def _edges(self):
        return np.r_[0., np.cumsum([abs(a.amount) for a in self.actions])]

    @cached_property
    def knots(self):
        return tuple(ControlKnot(float(s), a.control, 0.)
                     for s, a in zip(self._edges[:-1], self.actions)) + (
                         ControlKnot(self.length, self.actions[-1].control, 0.),)

    @cached_property
    def _knot_arrays(self):
        return tuple(np.asarray([getattr(k, name) for k in self.knots])
                     for name in ('s', 'value', 'derivative'))

    def control_many(self, stations):
        s = np.clip(np.asarray(stations, dtype=float), 0., self.length)
        idx = np.minimum(len(self.actions)-1,
                         np.searchsorted(self._edges[1:], s, side='right'))
        values = np.asarray([a.control for a in self.actions])[idx]
        return values, np.zeros_like(s), np.zeros_like(s)

    def control(self, s):
        return tuple(float(v) for v in self.control_many(s))

    def control_integral(self, stations):
        s = np.clip(np.asarray(stations, dtype=float), 0., self.length)
        idx = np.minimum(len(self.actions)-1,
                         np.searchsorted(self._edges[1:], s, side='right'))
        values = np.asarray([a.control for a in self.actions])
        prefix = np.r_[0., np.cumsum(np.diff(self._edges)*values)]
        return prefix[idx]+(s-self._edges[idx])*values[idx]


def exact_execution_plan(start, goal, actions):
    """Constant controls remain constant; no smoothing or endpoint fitting."""
    groups = []
    for a in collapse_exact_runs(list(actions)):
        if not all(math.isfinite(x) for x in (a.amount, a.control)):
            raise ValueError('nonfinite_execution_action')
        if abs(a.amount) <= 1e-10:
            continue
        if a.mode not in (0, 1, 2):
            raise ValueError('unknown_execution_mode')
        key = (a.mode, 1 if a.amount > 0 else -1)
        if not groups or groups[-1][0] != key:
            groups.append((key, []))
        groups[-1][1].append(a)
    blocks = []
    for (mode, direction), group in groups:
        if mode == 2:
            blocks.append(MotionBlock(2, direction, 0., (), sum(a.amount for a in group)))
        else:
            blocks.append(ExactControlBlock(mode, direction, tuple(group)))
    return ContinuousPlan(start, tuple(blocks), goal)


def time_actions(start, actions, limits, dt=.02, spatial_dt=.005, goal=None):
    actions = list(actions)
    if limits.translation_speed_policy != 'mode_switch_only':
        from .retiming import retime
        return None, retime(start, actions, limits, dt=dt, allow_reverse=True)
    plan = exact_execution_plan(start, goal or replay(start, actions)[-1], actions)
    return plan, retime_continuous(plan, limits, dt=dt, spatial_dt=spatial_dt)


def serialize_plan(plan):
    if plan is None:
        return None
    blocks = []
    for b in plan.blocks:
        row = {'mode': b.mode, 'direction': b.direction, 'length_m': b.length,
               'signed_angle_rad': b.signed_angle, 'slope_policy': b.slope_policy,
               'knots': [[k.s, k.value, k.derivative] for k in b.knots]}
        if isinstance(b, ExactControlBlock):
            row.update(representation='exact_piecewise_constant_v1',
                       actions=[[a.mode, a.amount, a.control] for a in b.actions])
        else:
            row['representation'] = 'continuous_hermite_v1'
        blocks.append(row)
    return blocks


def vehicle_contract(length, width, margin, limits):
    payload = {'schema': 'coptrans_vehicle_execution_v1',
               'length_m': float(length), 'width_m': float(width),
               'margin_total_dimension_m': float(margin),
               'initial_mode': 0, 'initial_speed_mps': 0., 'final_speed_mps': 0.,
               'limits': asdict(limits)}
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return {'sha256': hashlib.sha256(encoded.encode()).hexdigest(), **payload}


def restore_plan(start, goal, blocks):
    restored = []
    for b in blocks:
        if b.get('representation') == 'exact_piecewise_constant_v1':
            restored.append(ExactControlBlock(int(b['mode']), int(b['direction']),
                            tuple(Action(int(m), float(a), float(c)) for m,a,c in b['actions'])))
        else:
            restored.append(MotionBlock(int(b['mode']), int(b['direction']),
                float(b['length_m']), tuple(ControlKnot(*k) for k in b['knots']),
                float(b.get('signed_angle_rad', 0.)), b.get('slope_policy', 'fixed')))
    return ContinuousPlan(start, tuple(restored), goal)


def certify_display_sampling(plan, timed, evaluator, actions, max_levels=4):
    """Refine display sampling without changing geometry or execution time."""
    original_duration = timed.duration
    for level in range(max_levels+1):
        if all(evaluator.sweep_grid.interpolated_clear(a[1], b[1], .005)
               for a,b in zip(timed.samples,timed.samples[1:])):
            if level:
                audit = (evaluator.evaluate_continuous_geometry(plan, timed,
                            allow_reverse=evaluator.allow_reverse)
                         if plan is not None else
                         evaluator.evaluate(actions, timed, allow_reverse=evaluator.allow_reverse))
                if audit.get('status') != 'FEASIBLE':
                    raise ValueError('resampled_execution_audit:'+str(audit.get('reason')))
            else:
                audit = None
            return timed, audit, {'level': level, 'samples': len(timed.samples),
                'duration_change_s': timed.duration-original_duration,
                'geometry_changed': False, 'sweep_clear': True}
        if level == max_levels:
            break
        step = .005/(2**(level+1))
        if plan is not None:
            timed = retime_continuous(plan, evaluator.limits, dt=.02/(2**(level+1)),
                                      spatial_dt=step)
        else:
            _, timed = time_actions(evaluator.start, actions, evaluator.limits,
                                     dt=.02/(2**(level+1)), spatial_dt=step,
                                     goal=evaluator.goal)
        if abs(timed.duration-original_duration) > 1e-7:
            raise ValueError('display_sampling_changed_execution_time')
    raise ValueError('shared_display_sweep_collision_after_sampling_refinement')
