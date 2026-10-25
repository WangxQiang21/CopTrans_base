"""Velocity-profile QP for 4WIS Hybrid A* backend (K01 §6).

Given a geometric path
    {(x_i, y_i, theta_i, mode_i)  for i in 0..N}
with mode-switch indices K = {k : mode_k != mode_{k-1}} plus i=0 and i=N,
solve for v_i >= 0 that minimizes travel time under:

    v_0            = v_start          (usually 0)
    v_N            = v_goal           (usually 0)
    v_k            = 0                for every k in K       <- HARD constraint
    0 <= v_i       <= v_max(mode_i)
    | v_{i+1}^2 - v_i^2 | <= 2·a_max·Δs_i     (kinematic accel)
    v_i^2 · |κ_i|          <= a_lat_max        (centripetal limit)

Objective:
    J = Σ  2·Δs_i / (v_i + v_{i+1})                # trapezoidal time
      + λ · Σ  (v_{i+1} - v_i)^2 / Δs_i             # jerk-like smoothness

Solved with CasADi + IPOPT. Warm-startable from v_local frontend heuristic.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np
import casadi as ca


# Mode enum matching fwis_common/modes.hpp
MODE_ACKERMANN = 0
MODE_LATERAL = 1
MODE_DIAGONAL = 2
MODE_SPIN = 3


@dataclass
class QPConfig:
    v_max_ack: float = 1.50   # m/s (|delta|<20°, Ranger Mini datasheet p.20)
    v_max_ack_hi: float = 0.75  # m/s (|delta|>=20°)
    v_max_lat: float = 0.75   # m/s
    v_max_dia: float = 1.50   # m/s (|phi|<20°)
    v_max_dia_hi: float = 0.75  # m/s (|phi|>=20°)
    v_max_spin: float = 1.0   # rad/s (angular for Spin)
    a_max: float = 2.0        # m/s², longitudinal
    a_lat_max: float = 1.5    # m/s², centripetal
    lambda_jerk: float = 0.1  # weight on smoothness term
    v_start: float = 0.0
    v_goal: float = 0.0
    ipopt_print_level: int = 0
    ipopt_max_iter: int = 200


@dataclass
class QPResult:
    success: bool
    v: np.ndarray            # length N+1
    t: np.ndarray            # cumulative time at each waypoint
    total_time: float
    switch_indices: List[int] = field(default_factory=list)
    solver_status: str = ""


def _compute_curvature(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Discrete curvature κ_i via 3-point circumscribed circle. Endpoints=0."""
    n = len(x)
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        x1, y1 = x[i - 1], y[i - 1]
        x2, y2 = x[i], y[i]
        x3, y3 = x[i + 1], y[i + 1]
        a = math.hypot(x2 - x1, y2 - y1)
        b = math.hypot(x3 - x2, y3 - y2)
        c = math.hypot(x3 - x1, y3 - y1)
        s = 0.5 * (a + b + c)
        area_sq = max(0.0, s * (s - a) * (s - b) * (s - c))
        area = math.sqrt(area_sq)
        if area < 1e-9 or a * b * c < 1e-9:
            kappa[i] = 0.0
        else:
            r = (a * b * c) / (4.0 * area)
            kappa[i] = 1.0 / max(r, 1e-6)
    return kappa


def _v_max_at(mode: int, phi: float, cfg: QPConfig) -> float:
    """Datasheet-consistent per-mode speed cap."""
    if mode == MODE_ACKERMANN:
        return cfg.v_max_ack_hi if abs(phi) >= math.radians(20) else cfg.v_max_ack
    if mode == MODE_LATERAL:
        return cfg.v_max_lat
    if mode == MODE_DIAGONAL:
        return cfg.v_max_dia_hi if abs(phi) >= math.radians(20) else cfg.v_max_dia
    if mode == MODE_SPIN:
        return cfg.v_max_spin
    return cfg.v_max_ack


def find_switch_indices(modes: Sequence[int]) -> List[int]:
    """Indices i where mode_i != mode_{i-1}. Includes 0 and N as endpoints."""
    n = len(modes)
    out = [0]
    for i in range(1, n):
        if modes[i] != modes[i - 1]:
            out.append(i)
    if n - 1 not in out:
        out.append(n - 1)
    return out


def solve_velocity_qp(
    x: Sequence[float],
    y: Sequence[float],
    theta: Sequence[float],
    modes: Sequence[int],
    phis: Optional[Sequence[float]] = None,
    cfg: Optional[QPConfig] = None,
    v_warm: Optional[Sequence[float]] = None,
) -> QPResult:
    """Solve the velocity-profile QP for a given geometric path."""
    if cfg is None:
        cfg = QPConfig()

    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    theta = np.asarray(theta, dtype=float)
    modes = np.asarray(modes, dtype=int)
    n = len(x)
    if n < 2:
        return QPResult(success=False, v=np.zeros(n), t=np.zeros(n),
                        total_time=0.0, solver_status="path too short")

    if phis is None:
        phis = np.zeros(n)
    phis = np.asarray(phis, dtype=float)

    # Arc-length increments Δs_i between i and i+1.
    ds = np.hypot(np.diff(x), np.diff(y))
    # For Spin segments Δs is ~0 (position fixed); use yaw diff as pseudo-length
    for i in range(n - 1):
        if modes[i] == MODE_SPIN and modes[i + 1] == MODE_SPIN:
            ds[i] = max(ds[i], abs(theta[i + 1] - theta[i]) * 0.1)  # tiny nominal
        if ds[i] < 1e-6:
            ds[i] = 1e-6

    kappa = _compute_curvature(x, y)

    switch_idx = find_switch_indices(modes.tolist())

    opti = ca.Opti()
    v = opti.variable(n)

    # ---- Bounds ----
    for i in range(n):
        vm = _v_max_at(int(modes[i]), float(phis[i]), cfg)
        opti.subject_to(v[i] >= 0.0)
        opti.subject_to(v[i] <= vm)

    # ---- Hard constraints: v=0 at switch points (including start/end) ----
    for k in switch_idx:
        opti.subject_to(v[k] == 0.0)

    # ---- Kinematic longitudinal accel: |v_{i+1}^2 - v_i^2| <= 2·a·Δs ----
    for i in range(n - 1):
        opti.subject_to(v[i + 1] ** 2 - v[i] ** 2 <= 2.0 * cfg.a_max * ds[i])
        opti.subject_to(v[i] ** 2 - v[i + 1] ** 2 <= 2.0 * cfg.a_max * ds[i])

    # ---- Centripetal: v^2 · κ <= a_lat_max (only for Ackermann; L/P/S: κ~0) ----
    for i in range(n):
        if int(modes[i]) == MODE_ACKERMANN and kappa[i] > 1e-6:
            opti.subject_to(v[i] ** 2 * kappa[i] <= cfg.a_lat_max)

    # ---- Objective: trapezoidal time + jerk smoothness ----
    eps = 1e-3   # for numerical safety when v_i + v_{i+1} → 0
    J_time = 0
    J_jerk = 0
    for i in range(n - 1):
        J_time += 2.0 * ds[i] / (v[i] + v[i + 1] + eps)
        J_jerk += (v[i + 1] - v[i]) ** 2 / ds[i]
    opti.minimize(J_time + cfg.lambda_jerk * J_jerk)

    # ---- Warm start ----
    if v_warm is not None and len(v_warm) == n:
        opti.set_initial(v, np.asarray(v_warm, dtype=float))
    else:
        # Simple warm start: fill v_max everywhere, then zero out switches.
        v0 = np.array([_v_max_at(int(modes[i]), float(phis[i]), cfg) * 0.5
                       for i in range(n)])
        for k in switch_idx:
            v0[k] = 0.0
        opti.set_initial(v, v0)

    # ---- Solve ----
    ipopt_opts = {
        "print_level": cfg.ipopt_print_level,
        "sb": "yes",
        "max_iter": cfg.ipopt_max_iter,
        "tol": 1e-6,
        "acceptable_tol": 1e-4,
    }
    opti.solver("ipopt", {"print_time": False}, ipopt_opts)

    try:
        sol = opti.solve()
        v_opt = np.array(sol.value(v))
        status = sol.stats().get("return_status", "unknown")
    except Exception as e:
        # IPOPT can return partial solution on non-fatal termination.
        try:
            v_opt = np.array(opti.debug.value(v))
            status = f"non-optimal ({e})"
        except Exception:
            return QPResult(success=False, v=np.zeros(n), t=np.zeros(n),
                            total_time=0.0, switch_indices=switch_idx,
                            solver_status=f"failed: {e}")

    # Cumulative time
    t = np.zeros(n)
    for i in range(n - 1):
        t[i + 1] = t[i] + 2.0 * ds[i] / max(v_opt[i] + v_opt[i + 1], eps)

    return QPResult(
        success=("Solve_Succeeded" in status or "Solved" in status
                 or "optimal" in status.lower()),
        v=v_opt,
        t=t,
        total_time=float(t[-1]),
        switch_indices=switch_idx,
        solver_status=status,
    )
