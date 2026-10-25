#!/usr/bin/env python3
"""Corridor-constrained low-speed 4WIS trajectory optimizer.

The input is a topology channel, not a pre-labelled motion path.  Optimisation
uses the body twist (u, v, r), allowing longitudinal/Ackermann, lateral, crab
and spin behaviour to emerge continuously.  Mode labels are recovered from the
optimised twist only when serialising the result.
"""
import math
import sys

import casadi as ca
import numpy as np


def read_input(path):
    tokens = open(path, encoding="utf-8").read().split()
    at = 0
    def expect(name):
        nonlocal at
        if at >= len(tokens) or tokens[at] != name:
            raise RuntimeError("expected %s" % name)
        at += 1
    expect("CONFIG")
    names = ("robot_length", "wheelbase", "track_width", "max_wheel_steer_rate", "enforce_wheel_rate_in_ocp", "lock_verified_geometry",
             "max_velocity", "max_acceleration",
             "max_yaw_rate", "max_yaw_acceleration", "comfort_weight",
             "tracking_weight", "ipopt_max_iterations", "ipopt_tol",
             "infeasibility_tolerance")
    cfg = {}
    for name in names:
        cfg[name] = float(tokens[at]); at += 1
    cfg["ipopt_max_iterations"] = int(cfg["ipopt_max_iterations"])
    expect("POINTS")
    count = int(tokens[at]); at += 1
    points = []
    for _ in range(count):
        points.append((float(tokens[at]), float(tokens[at + 1]), float(tokens[at + 2])))
        at += 3
    def read_corridors(name):
        nonlocal at
        expect(name)
        count = int(tokens[at]); at += 1
        rows = []
        for _ in range(count):
            rows.append(tuple(float(tokens[at + j]) for j in range(10)))
            at += 10
        return rows
    return cfg, points, read_corridors("FRONT_CORRIDORS"), read_corridors("REAR_CORRIDORS")


def corridor_constraint(opti, px, py, row):
    cx, cy, tx, ty, nx, ny, bwd, fwd, right, left = row
    dx, dy = px - cx, py - cy
    s, l = dx * tx + dy * ty, dx * nx + dy * ny
    opti.subject_to(s >= -bwd)
    opti.subject_to(s <= fwd)
    opti.subject_to(l >= -right)
    opti.subject_to(l <= left)


def unwrap(yaw):
    return np.unwrap(np.asarray(yaw, dtype=float))


def solve_all(cfg, points, front, rear):
    npts = len(points)
    if npts < 3:
        return [(x, y, t, 0., 0., 0.) for x, y, t in points], 0., False, -2
    n = npts - 1
    xr = np.array([p[0] for p in points])
    yr = np.array([p[1] for p in points])
    tr = unwrap([p[2] for p in points])
    length = float(np.hypot(np.diff(xr), np.diff(yr)).sum())
    t_guess = max(length / max(cfg["max_velocity"] * 0.7, 1e-3), 3.0)
    h_guess = t_guess / n

    # Raw-channel finite differences form a feasible warm start in the body
    # frame.  The NLP may then redistribute it into lateral/crab segments.
    wx = np.gradient(xr, h_guess)
    wy = np.gradient(yr, h_guess)
    ug = wx * np.cos(tr) + wy * np.sin(tr)
    vg = -wx * np.sin(tr) + wy * np.cos(tr)
    rg = np.gradient(tr, h_guess)
    speed = np.hypot(ug, vg)
    scale = np.minimum(1.0, cfg["max_velocity"] / np.maximum(speed, 1e-6))
    ug, vg = ug * scale, vg * scale
    rg = np.clip(rg, -cfg["max_yaw_rate"], cfg["max_yaw_rate"])
    ug[[0, -1]] = 0.; vg[[0, -1]] = 0.; rg[[0, -1]] = 0.

    opti = ca.Opti()
    # x, y, yaw, body longitudinal velocity, lateral velocity, yaw rate
    X = opti.variable(6, npts)
    # body longitudinal/lateral acceleration, yaw acceleration
    U = opti.variable(3, n)
    T = opti.variable()
    x, y, th, u, v, r = [X[i, :] for i in range(6)]
    h = T / n
    objective = T
    for k in range(n):
        dx = u[k] * ca.cos(th[k]) - v[k] * ca.sin(th[k])
        dy = u[k] * ca.sin(th[k]) + v[k] * ca.cos(th[k])
        opti.subject_to(x[k + 1] == x[k] + dx * h)
        opti.subject_to(y[k + 1] == y[k] + dy * h)
        opti.subject_to(th[k + 1] == th[k] + r[k] * h)
        opti.subject_to(u[k + 1] == u[k] + U[0, k] * h)
        opti.subject_to(v[k + 1] == v[k] + U[1, k] * h)
        opti.subject_to(r[k + 1] == r[k] + U[2, k] * h)
        objective += cfg["comfort_weight"] * (
            U[0, k] ** 2 + U[1, k] ** 2 + 0.3 * U[2, k] ** 2) * h

    half_axis = 0.25 * cfg["robot_length"]
    for k in range(npts):
        xf = x[k] + half_axis * ca.cos(th[k])
        yf = y[k] + half_axis * ca.sin(th[k])
        xb = x[k] - half_axis * ca.cos(th[k])
        yb = y[k] - half_axis * ca.sin(th[k])
        corridor_constraint(opti, xf, yf, front[k])
        corridor_constraint(opti, xb, yb, rear[k])
        objective += cfg["tracking_weight"] * (
            (x[k] - xr[k]) ** 2 + (y[k] - yr[k]) ** 2) * h
        opti.subject_to(u[k] ** 2 + v[k] ** 2 <= cfg["max_velocity"] ** 2)
        opti.subject_to(r[k] >= -cfg["max_yaw_rate"])
        opti.subject_to(r[k] <= cfg["max_yaw_rate"])
    for k in range(n):
        opti.subject_to(U[0, k] >= -cfg["max_acceleration"])
        opti.subject_to(U[0, k] <= cfg["max_acceleration"])
        opti.subject_to(U[1, k] >= -cfg["max_acceleration"])
        opti.subject_to(U[1, k] <= cfg["max_acceleration"])
        opti.subject_to(U[2, k] >= -cfg["max_yaw_acceleration"])
        opti.subject_to(U[2, k] <= cfg["max_yaw_acceleration"])

    # Exact low-speed rigid-body wheel-heading rate constraint.  This makes
    # the optimiser slow down locally through T/u/v/r instead of relying on a
    # global post-processing time stretch.  The current chassis operates in
    # the non-wrapping +/- pi/2 steering range; the post-solve validator still
    # covers the reversible-drive equivalent-angle case.
    if cfg["enforce_wheel_rate_in_ocp"] >= 0.5:
        half_wb = 0.5 * cfg["wheelbase"]
        half_track = 0.5 * cfg["track_width"]
        for k in range(n):
            for px, py in ((half_wb, half_track), (half_wb, -half_track),
                           (-half_wb, half_track), (-half_wb, -half_track)):
                delta0 = ca.atan2(v[k] + r[k] * px, u[k] - r[k] * py)
                delta1 = ca.atan2(v[k + 1] + r[k + 1] * px,
                                  u[k + 1] - r[k + 1] * py)
                opti.subject_to(delta1 - delta0 <= cfg["max_wheel_steer_rate"] * h)
                opti.subject_to(delta1 - delta0 >= -cfg["max_wheel_steer_rate"] * h)

    for i, value in enumerate((xr[0], yr[0], tr[0], 0., 0., 0.)):
        opti.subject_to(X[i, 0] == value)
    for i, value in enumerate((xr[-1], yr[-1], tr[-1], 0., 0., 0.)):
        opti.subject_to(X[i, -1] == value)
    if cfg["lock_verified_geometry"] >= 0.5:
        # The topology seed has already passed dense full-OBB sweep checking.
        # Until the corridor represents the full footprint, do not let the NLP
        # alter (x,y,theta) and invalidate that guarantee.
        for k in range(npts):
            opti.subject_to(x[k] == xr[k])
            opti.subject_to(y[k] == yr[k])
            opti.subject_to(th[k] == tr[k])
    opti.subject_to(T >= 1e-3)
    opti.set_initial(X, np.vstack((xr, yr, tr, ug, vg, rg)))
    opti.set_initial(U, np.vstack((np.diff(ug) / h_guess, np.diff(vg) / h_guess,
                                   np.diff(rg) / h_guess)))
    opti.set_initial(T, t_guess)
    opti.minimize(objective)
    opti.solver("ipopt", {"print_time": False}, {
        "print_level": 0, "max_iter": cfg["ipopt_max_iterations"],
        "tol": max(cfg["ipopt_tol"], 1e-8),
        "acceptable_tol": max(cfg["infeasibility_tolerance"], cfg["ipopt_tol"], 1e-7)})
    try:
        sol = opti.solve()
        xs = sol.value(X); total = float(sol.value(T)); ok, status = True, 0
    except RuntimeError:
        xs = opti.debug.value(X); total = float(opti.debug.value(T)); ok, status = False, -1
    states = [(float(xs[0, k]), float(xs[1, k]),
               (float(xs[2, k]) + math.pi) % (2 * math.pi) - math.pi,
               float(xs[3, k]), float(xs[4, k]), float(xs[5, k]))
              for k in range(npts)]
    return states, total, ok, status


def write_output(path, states, total, ok, status):
    with open(path, "w", encoding="utf-8") as f:
        f.write("STATUS %d %d %.17g\n" % (1 if ok else 0, status, total))
        f.write("STATES %d\n" % len(states))
        for state in states:
            f.write("%.17g %.17g %.17g %.17g %.17g %.17g\n" % state)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: casadi_safe_corridor_backend.py <input> <output>")
    write_output(sys.argv[2], *solve_all(*read_input(sys.argv[1])))
