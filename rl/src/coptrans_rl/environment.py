"""Gymnasium planning environment; decision steps are not physical stops."""
from dataclasses import dataclass
import math
import numbers
import time
from types import SimpleNamespace

import gymnasium as gym
import numpy as np

from .action_library import ActionLibrary
from .action_mask import valid_actions
from .kernel_adapter import (
    advance, wrap, FinalizationConfig, finalize_candidate)
from .observation import ObservationBuilder
from .reward import proxy_seconds, shaped_step, success_correction


@dataclass(frozen=True)
class EnvironmentConfig:
    max_decisions: int = 256
    time_scale: float = 10.
    success_bonus: float = 50.
    failure_penalty: float = 50.
    potential_weight: float = 1.
    max_execution_seconds: float = 300.
    finalization_timeout_s: float = 3.
    image_size: int = 64

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, numbers.Integral)
               for v in (self.max_decisions, self.image_size)):
            raise ValueError("environment_sizes_must_be_integers")
        if self.max_decisions < 1 or self.image_size < 8:
            raise ValueError("invalid_environment_size")
        for value in (self.time_scale, self.success_bonus, self.failure_penalty,
                      self.max_execution_seconds, self.finalization_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid_environment_limit")
        if not math.isfinite(self.potential_weight) or self.potential_weight < 0:
            raise ValueError("invalid_potential_weight")


class PlanningEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, problem_factory, library, config=None,
                 finalization_config=None):
        super().__init__()
        self.problem_factory, self.library = problem_factory, library
        self.config = config or EnvironmentConfig()
        self.finalization_config = finalization_config or FinalizationConfig(
            include_metrics=False)
        n = self.config.image_size
        self.action_space = gym.spaces.Discrete(len(library))
        self.observation_space = gym.spaces.Dict({
            "local_map": gym.spaces.Box(0., 1., (4, n, n), np.float32),
            "global_map": gym.spaces.Box(0., 1., (5, n, n), np.float32),
            "state": gym.spaces.Box(-1000., 1000., (60,), np.float32),
            "valid_action_mask": gym.spaces.Box(0., 1., (len(library),), np.float32)})
        self.done = True
        self.last_finalization = None

    def _at_goal(self):
        return (math.hypot(self.pose.x-self.problem.goal.x,
                           self.pose.y-self.problem.goal.y) <= self.problem.xy_tol and
                abs(wrap(self.pose.th-self.problem.goal.th)) <= self.problem.yaw_tol)

    def _observe(self):
        return self.builder.encode(self.pose, self.actions, self.steps,
                                   self.config.max_decisions, self.proxy_elapsed,
                                   self._mask)

    def _update_mask(self):
        self._mask, diag = valid_actions(
            self.problem, self.pose, self.actions, self.library)
        self.mask_seconds += diag["wall_s"]
        self.mask_diagnostics = diag

    def action_masks(self):
        return self._mask.copy()

    def diagnostics(self):
        # Wall-clock measurements live outside Gym transition info so that
        # seeded transitions, including info, remain reproducible.
        output = {"mask_wall_s": self.mask_seconds}
        if self.last_finalization is not None:
            output["finalize_wall_s"] = self.last_finalization.stages_s["total"]
        return output

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        problem = options.get("problem") if options else None
        self.problem = problem or self.problem_factory(self.np_random)
        expected = ActionLibrary.from_contract(SimpleNamespace(
            length=self.problem.length, limits=self.problem.limits))
        if expected.identity != self.library.identity:
            raise ValueError("action_library_vehicle_contract_mismatch")
        if self.problem.initial_mode != 0:
            raise ValueError("unsupported_initial_mode")
        for pose in (self.problem.start, self.problem.goal):
            if (not all(math.isfinite(v) for v in (pose.x, pose.y, pose.th)) or
                    not self.problem.grid.pose_clear(pose)):
                raise ValueError("invalid_start_or_goal_footprint")
        self.pose = self.problem.start
        self.actions, self.steps, self.proxy_elapsed = [], 0, 0.
        self.done, self.last_finalization = False, None
        self.mask_seconds = 0.
        if self._at_goal():
            self.done = True
            raise ValueError("zero_motion_task_requires_direct_finalization")
        self.builder = ObservationBuilder(self.problem, self.config.image_size)
        self._update_mask()
        if not self._mask.any():
            self.done = True
            raise ValueError("no_legal_action_at_reset")
        return self._observe(), {"task_id": self.problem.task_id,
                                 "action_library_sha256": self.library.identity}

    def _fail(self, reason, old_potential, proxy=0.):
        self.done = True
        reward = shaped_step(proxy, old_potential, 0., self.config.time_scale,
                             self.config.potential_weight)-self.config.failure_penalty
        # No sampling happens after a terminated state, but retaining the last
        # mask avoids feeding all-invalid logits to terminal value evaluation.
        return self._observe(), reward, True, False, {
            "is_success": False, "reason": reason, "decisions": self.steps,
            "proxy_seconds": self.proxy_elapsed}

    def step(self, action_id):
        if self.done:
            raise RuntimeError("step_after_episode_end")
        old_potential = self.builder.potential(self.pose)
        try:
            action = self.library[action_id]
        except ValueError:
            return self._fail("invalid_action_id", old_potential)
        if not self._mask[int(action_id)]:
            return self._fail("masked_action_selected", old_potential)
        previous_mode = self.actions[-1].mode if self.actions else self.problem.initial_mode
        proxy = proxy_seconds(action, previous_mode, self.problem.limits)
        self.proxy_elapsed += proxy
        self.actions.append(action)
        self.steps += 1
        self.pose = advance(self.pose, action)
        self.builder.record(self.pose)
        if self._at_goal():
            self.done = True
            self.last_finalization = finalize_candidate(
                self.problem, self.actions, self.finalization_config,
                time.perf_counter()+self.config.finalization_timeout_s,
                source="rl_rollout")
            result = self.last_finalization
            if not result.feasible:
                return self._fail("finalization:"+str(result.reason),
                                  old_potential, proxy)
            if result.timed.duration > self.config.max_execution_seconds:
                return self._fail("execution_time_budget_exceeded",
                                  old_potential, proxy)
            reward = shaped_step(proxy, old_potential, 0., self.config.time_scale,
                                 self.config.potential_weight)
            reward += success_correction(self.proxy_elapsed, result.timed.duration,
                                          self.config.time_scale,
                                          self.config.success_bonus)
            return self._observe(), reward, True, False, {
                "is_success": True, "reason": "certified_goal",
                "execution_seconds": result.timed.duration, "decisions": self.steps,
                "proxy_seconds": self.proxy_elapsed}
        if self.steps >= self.config.max_decisions:
            return self._fail("decision_budget_exceeded", old_potential, proxy)
        self._update_mask()
        if not self._mask.any():
            return self._fail("no_legal_action", old_potential, proxy)
        reward = shaped_step(proxy, old_potential, self.builder.potential(self.pose),
                             self.config.time_scale, self.config.potential_weight)
        return self._observe(), reward, False, False, {"is_success": False}
