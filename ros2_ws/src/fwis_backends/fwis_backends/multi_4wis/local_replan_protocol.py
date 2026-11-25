"""Offline coordination prototype for a certified local three-car window."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, replace
import hashlib
import json
import math
from threading import Event
from time import monotonic_ns

import numpy as np

from .core import Grid
from .uniform_motion import PRIMITIVE_A_ARC, PRIMITIVE_S_SPIN, UniformMotionEdge
from .uniform_pipeline import plan_uniform, request_fingerprint
from .uniform_timing import MotionTiming, SetupTiming
from .uniform_trajectory import UniformTimedPlan


_NS = 1_000_000_000
_IDS = (1, 2, 3)


def _grid_digest(grid_record):
    payload = json.dumps(grid_record, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True)
class MapRevision:
    number: int
    grid_sha256: str

    @classmethod
    def observe(cls, number, request):
        if isinstance(number, bool) or not isinstance(number, int) or number < 0:
            raise ValueError("map revision must be a nonnegative integer")
        return cls(number, _grid_digest(request_fingerprint(request)[1]["grid"]))


@dataclass(frozen=True)
class CertifiedPlan:
    plan: UniformTimedPlan
    revision: MapRevision
    epoch_ns: int
    model_sha256: str
    request_sha256: str

    @classmethod
    def capture(cls, outcome, revision, epoch_ns):
        audit = outcome.evaluation
        if (not outcome.executable or not isinstance(outcome.plan, UniformTimedPlan)
                or outcome.plan.vehicle_ids != _IDS or audit is None or not audit.independent_audit
                or not audit.collision_certificate
                or audit.metrics.get("dynamic_continuous_certificate") is not True
                or audit.metrics.get("map_version_checked") is not True
                or not outcome.request_inputs.get("grid")
                or not outcome.request_sha256
                or _grid_digest(outcome.request_inputs["grid"]) != revision.grid_sha256
                or not isinstance(epoch_ns, int) or isinstance(epoch_ns, bool) or epoch_ns < 0):
            raise ValueError("expected a certified three-car UniformTimedPlan for this map and epoch")
        digest = outcome.plan.model_sha256()
        if (audit.plan_id != outcome.plan.plan_id or audit.metrics.get("model_sha256") != digest
                or outcome.projection is None or outcome.projection.model_sha256 != digest):
            raise ValueError("candidate model, projection and independent audit differ")
        return cls(outcome.plan, revision, epoch_ns, digest, outcome.request_sha256)

    @property
    def end_ns(self):
        return self.epoch_ns + round(self.plan.duration_s * _NS)


@dataclass(frozen=True)
class LocalWindow:
    parent_plan_id: int
    parent_sha256: str
    stop_ns: int
    epoch_ns: int
    stop_end_ns: int
    start_edge: UniformMotionEdge
    goal_edge: UniformMotionEdge
    max_path_m: float
    max_motion_s: float


def _path_length(edges):
    return sum(0.0 if edge.primitive == PRIMITIVE_S_SPIN else
               edge.control.parameter_length * math.hypot(edge.control.pivot_x, edge.control.pivot_y)
               if edge.primitive == PRIMITIVE_A_ARC else edge.control.parameter_length
               for edge in edges)


def stopped_window(snapshot, now_ns, *, min_lead_ns, switch_dwell_ns,
                   max_path_m, max_motion_s, max_lookahead_ns):
    """Use a future existing stationary phase and a bounded stopped endpoint."""
    if (min_lead_ns <= 0 or switch_dwell_ns <= 0 or max_lookahead_ns <= min_lead_ns
            or not math.isfinite(max_path_m) or max_path_m <= 0.0
            or not math.isfinite(max_motion_s) or max_motion_s <= 0.0):
        raise ValueError("window horizons must be finite and positive")
    phases = snapshot.plan.phases
    for index, (previous, setup) in enumerate(zip(phases, phases[1:]), 1):
        if not (isinstance(previous, MotionTiming) and isinstance(setup, SetupTiming)):
            continue
        stop = snapshot.epoch_ns + round(setup.t0_s * _NS)
        epoch = stop + switch_dwell_ns
        stop_end = snapshot.epoch_ns + round((setup.t0_s + setup.duration_s) * _NS)
        if stop < now_ns + min_lead_ns or epoch > stop_end or stop > now_ns + max_lookahead_ns:
            continue
        for following in phases[index + 1:]:
            if not isinstance(following, MotionTiming):
                continue
            duration = following.end_time - setup.t0_s
            if duration <= max_motion_s and _path_length((following.edge,)) <= max_path_m:
                return LocalWindow(snapshot.plan.plan_id, snapshot.model_sha256, stop, epoch,
                                   stop_end, previous.edge, following.edge, max_path_m, max_motion_s)
            break
    return None


@dataclass(frozen=True)
class Preparation:
    plan_id: int
    model_sha256: str
    revision: MapRevision
    epoch_ns: int
    expected_ids: tuple = _IDS


@dataclass(frozen=True)
class Acknowledgement:
    vehicle_id: int
    preparation: Preparation
    stamp_ns: int
    stopped: bool


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str
    preparation: Preparation = None


def _pose_matches(first, second):
    return (math.hypot(first.x - second.x, first.y - second.y) <= 1e-9
            and abs(math.remainder(first.psi - second.psi, math.tau)) <= 1e-9)


def _headings_match(first, second):
    a, b = dict(first), dict(second)
    return a.keys() == b.keys() and all(abs(math.remainder(a[i] - b[i], math.tau)) <= 1e-9 for i in a)


class LocalReplanCoordinator:
    """Single-owner offline protocol. Poll never waits on the planner worker."""

    def __init__(self, active, *, clock=monotonic_ns, ack_freshness_ns=200_000_000,
                 release_margin_ns=100_000_000, planner=plan_uniform):
        if ack_freshness_ns <= 0 or release_margin_ns <= 0:
            raise ValueError("ack freshness and release margin must be positive")
        self.active = active
        self.revision = active.revision
        self._clock = clock
        self._ack_freshness_ns = ack_freshness_ns
        self._release_margin_ns = release_margin_ns
        self._planner = planner
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="formation-window")
        self._cancel = Event()
        self._last_ns = self._clock()
        self._job = self._window = self._next = self._preparation = None
        self._retired_job = None
        self._acks = {}
        self._committed = False
        self._halted = False
        self._used_plan_id = active.plan.plan_id
        self._reason = "GLOBAL_PLAN_ACTIVE"

    def _clear(self):
        self._cancel.set()
        if self._job is not None and not self._job.cancel() and not self._job.done():
            self._retired_job = self._job
        self._job = self._window = self._next = self._preparation = None
        self._acks.clear()
        self._committed = False

    def observe_map(self, new_revision):
        if new_revision != self.revision:
            self.revision = new_revision
            self._halted = True
            self._reason = "MAP_REVISION_CHANGED"
            self._clear()

    def begin(self, request, window, *, plan_id, computation_budget_s, max_expansions):
        now = self._clock()
        if self._retired_job is not None:
            if not self._retired_job.done():
                raise ValueError("previous planner worker is still exiting")
            self._retired_job = None
        if (self._halted or self._window is not None or self.active.revision != self.revision
                or window.parent_plan_id != self.active.plan.plan_id
                or window.parent_sha256 != self.active.model_sha256
                or not self.active.epoch_ns <= now < window.stop_ns < window.epoch_ns <= window.stop_end_ns
                or window.epoch_ns - window.stop_ns <= self._release_margin_ns
                or not isinstance(plan_id, int) or isinstance(plan_id, bool) or plan_id <= self._used_plan_id
                or plan_id > 2**32 - 1 or not math.isfinite(computation_budget_s)
                or computation_budget_s <= 0.0 or not isinstance(max_expansions, int)
                or isinstance(max_expansions, bool) or max_expansions <= 0):
            raise ValueError("local window, ID or budget cannot be admitted")
        # Revalidate the map and stationary handover against the actual active model.
        request_sha256, request_inputs = request_fingerprint(request)
        if (request_sha256 != self.active.request_sha256
                or _grid_digest(request_inputs["grid"]) != self.revision.grid_sha256):
            raise ValueError("request differs from the active certified inputs")
        phases = self.active.plan.phases
        matches = [
            index for index, (previous, setup) in enumerate(zip(phases, phases[1:]), 1)
            if (isinstance(previous, MotionTiming) and isinstance(setup, SetupTiming)
                and previous.edge is window.start_edge
                and self.active.epoch_ns + round(setup.t0_s * _NS) == window.stop_ns
                and self.active.epoch_ns + round((setup.t0_s + setup.duration_s) * _NS) == window.stop_end_ns)
        ]
        if len(matches) != 1:
            raise ValueError("window splice must be an existing stationary phase")
        subsequent = next((phase for phase in phases[matches[0] + 1:]
                           if isinstance(phase, MotionTiming)), None)
        if (subsequent is None or subsequent.edge is not window.goal_edge
                or not math.isfinite(window.max_path_m) or _path_length((subsequent.edge,)) > window.max_path_m
                or not math.isfinite(window.max_motion_s)
                or subsequent.end_time - phases[matches[0]].t0_s > window.max_motion_s):
            raise ValueError("local goal must be the next bounded common stop")
        self._used_plan_id = plan_id
        self._window = window
        self._reason = "COMPUTING_LOCAL_WINDOW"
        self._cancel = Event()
        cancel = self._cancel
        self._planning_deadline_ns = min(window.stop_ns, now + int(computation_budget_s * _NS))
        deadline = self._planning_deadline_ns
        revision = self.revision
        solver = self._planner
        clock = self._clock
        limits = replace(request.search_config, max_expansions=max_expansions, max_seconds=computation_budget_s,
                         continuous_a_refinement=False, prefer_ackermann=False,
                         convex_corridor=False, global_translation_route=False,
                         direct_goal_translation=True)
        occupancy = np.array(request.grid.occ, copy=True, order="C")
        occupancy.setflags(write=False)
        bounded = replace(
            request, grid=Grid(occupancy, request.grid.res, request.grid.ox, request.grid.oy),
            vehicle_shapes=dict(request.vehicle_shapes),
            plan_id=plan_id, initial_edge=window.start_edge, search_config=limits,
            start_qf=window.start_edge.end_qf, start_headings=dict(window.start_edge.end_headings),
            goal_qf=window.goal_edge.end_qf, goal_headings=dict(window.goal_edge.end_headings),
            successor_config=replace(request.successor_config, goal_heading_free=False,
                                     goal_formation_heading_free=False),
            reachability_retime=False, cancelled=lambda: cancel.is_set() or clock() >= deadline)

        def work():
            outcome = solver(bounded)
            if not outcome.executable:
                return None, outcome.reason
            if outcome.request_sha256 != request_fingerprint(bounded)[0]:
                return None, "LOCAL_WINDOW_REQUEST_MISMATCH"
            candidate = CertifiedPlan.capture(outcome, revision, window.epoch_ns)
            plan = candidate.plan
            if (plan.plan_id != plan_id or candidate.model_sha256 == window.parent_sha256
                    or not _pose_matches(plan.start_qf, bounded.start_qf)
                    or not _headings_match(plan.start_headings, bounded.start_headings)
                    or not _pose_matches(plan.goal_qf, bounded.goal_qf)
                    or not _headings_match(plan.goal_headings, bounded.goal_headings)
                    or plan.initial_edge is not window.start_edge or plan.duration_s > window.max_motion_s
                    or _path_length(plan.edges) > window.max_path_m):
                return None, "LOCAL_WINDOW_CERTIFICATE_MISMATCH"
            return candidate, "LOCAL_WINDOW_CERTIFIED"

        self._job = self._executor.submit(work)

    def acknowledge(self, ack):
        now = self._clock()
        if (self._halted or self._committed or self._preparation is None or now < self._last_ns
                or not isinstance(ack, Acknowledgement) or type(ack.vehicle_id) is not int
                or type(ack.stamp_ns) is not int or ack.vehicle_id not in _IDS or ack.stopped is not True
                or ack.preparation != self._preparation
                or not self._window.stop_ns <= ack.stamp_ns <= now
                or now - ack.stamp_ns > self._ack_freshness_ns
                or now >= self._window.epoch_ns - self._release_margin_ns):
            return False
        latest = self._acks.get(ack.vehicle_id)
        if latest is not None and ack.stamp_ns <= latest:
            return False
        self._acks[ack.vehicle_id] = ack.stamp_ns
        return True

    def _fallback(self, now):
        if (self._halted or self.active.revision != self.revision
                or not self.active.epoch_ns <= now < self.active.end_ns):
            return Decision("GROUP_STOP", self._reason if self._halted else "NO_VALID_ACTIVE_PLAN")
        return Decision("HOLD_OLD", self._reason)

    def poll(self):
        now = self._clock()
        if now < self._last_ns:
            self._clear()
            self._halted = True
            self._reason = "CLOCK_JUMP"
        self._last_ns = now
        if self._halted:
            return self._fallback(now)
        if self._committed:
            if now >= self._window.epoch_ns:
                self.active = self._next
                self._clear()
                self._reason = "LOCAL_PLAN_ACTIVE"
                if now >= self.active.end_ns:
                    return Decision("GROUP_STOP", "LOCAL_PLAN_COMPLETE")
                return Decision("EXECUTE_ACTIVE", "LOCAL_PLAN_ACTIVE")
            return Decision("HOLD_OLD", "WAIT_SHARED_EPOCH", self._preparation)
        if self._job is not None:
            if now >= self._planning_deadline_ns:
                self._clear()
                self._reason = "PLANNER_TIMEOUT"
            elif self._job.done():
                try:
                    candidate, self._reason = self._job.result()
                except Exception:
                    candidate, self._reason = None, "LOCAL_PLANNER_ERROR"
                self._job = None
                if candidate is None:
                    self._clear()
                else:
                    self._next = candidate
                    self._preparation = Preparation(candidate.plan.plan_id, candidate.model_sha256,
                                                    candidate.revision, candidate.epoch_ns)
        if self._preparation is not None:
            if now >= self._window.epoch_ns - self._release_margin_ns:
                self._clear()
                self._reason = "ACK_DEADLINE"
            elif (self._acks.keys() == set(_IDS) and all(
                    0 <= now - stamp <= self._ack_freshness_ns for stamp in self._acks.values())):
                self._committed = True
                return Decision("SWITCH", "ALL_VEHICLES_ACKNOWLEDGED", self._preparation)
            else:
                return Decision("PREPARE", "AWAITING_COMMON_STOP_AND_ACKS", self._preparation)
        if self._reason == "LOCAL_PLAN_ACTIVE" and now < self.active.end_ns:
            return Decision("EXECUTE_ACTIVE", self._reason)
        return self._fallback(now)

    def close(self):
        """Shutdown is an offline teardown step, outside the polling loop."""
        self._clear()
        self._executor.shutdown(wait=True, cancel_futures=True)
