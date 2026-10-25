"""Ideal geometric motion models and conservative raster footprint checks.

This module is an experiment prototype. It models perfect tracking, not actuator
dynamics or wheel-level inverse kinematics.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
from scipy.ndimage import binary_dilation


@dataclass(frozen=True)
class Vehicle:
    length: float = 0.738
    width: float = 0.500
    min_turn_radius: float = 0.810
    oblique_limit: float = math.pi / 2


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


def wrap(yaw: float) -> float:
    return (yaw + math.pi) % (2 * math.pi) - math.pi


def propagate(pose: Pose, mode: str, amount: float, control: float,
              vehicle: Vehicle) -> Pose:
    """Exact constant-control spatial propagation.

    A: amount is signed arc length; control is signed curvature (1/m).
    P/L: amount is signed translation length; control is body-relative angle.
    S: amount is signed yaw increment; control is unused.
    The planning reference point's Ackermann radius is provisionally set to
    the manual's 0.810 m minimum-radius value.
    """
    if mode == "A":
        k = control
        if abs(k) < 1e-10:
            return Pose(pose.x + amount * math.cos(pose.yaw),
                        pose.y + amount * math.sin(pose.yaw), pose.yaw)
        yaw2 = pose.yaw + amount * k
        return Pose(pose.x + (math.sin(yaw2) - math.sin(pose.yaw)) / k,
                    pose.y - (math.cos(yaw2) - math.cos(pose.yaw)) / k,
                    wrap(yaw2))
    if mode in ("P", "L"):
        angle = math.copysign(math.pi / 2, control) if mode == "L" else control
        yaw_motion = pose.yaw + angle
        return Pose(pose.x + amount * math.cos(yaw_motion),
                    pose.y + amount * math.sin(yaw_motion), pose.yaw)
    if mode == "S":
        return Pose(pose.x, pose.y, wrap(pose.yaw + amount))
    raise ValueError(f"unknown motion mode {mode!r}")


class RasterWorld:
    """Occupancy grid plus rasterized rectangular vehicle footprint checker."""
    def __init__(self, map_yaml: str, vehicle: Vehicle):
        meta = yaml.safe_load(Path(map_yaml).read_text())
        image_path = Path(map_yaml).parent / meta["image"]
        image = np.asarray(Image.open(image_path).convert("L"))
        # Inflate occupied map cells by one-cell Moore neighborhood (2.8 cm
        # diagonal); together with footprint sampling this protects against
        # gaps between rasterized footprint points.
        self.occupied = binary_dilation(image <= 65, structure=np.ones((3, 3), dtype=bool))
        self.res = float(meta["resolution"])
        self.ox, self.oy, self.origin_yaw = map(float, meta["origin"])
        self.height, self.width = self.occupied.shape
        self.vehicle = vehicle
        # Densely sample the entire footprint, including edges. Sampling is
        # finer than the occupancy resolution, making the check conservative
        # for the current static map representation.
        # Small conservative expansion covers quantized center/yaw cache error.
        cache_margin = 0.015
        x_extent = vehicle.length / 2 + cache_margin
        y_extent = vehicle.width / 2 + cache_margin
        xs = np.linspace(-x_extent, x_extent, math.ceil(2*x_extent/0.03)+1)
        ys = np.linspace(-y_extent, y_extent, math.ceil(2*y_extent/0.03)+1)
        self.fx, self.fy = np.meshgrid(xs, ys)
        self.fx, self.fy = self.fx.ravel(), self.fy.ravel()
        self._rotated_offsets = []
        for deg in range(360):
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            self._rotated_offsets.append((c*self.fx-s*self.fy, s*self.fx+c*self.fy))
        self._pose_clear_cached = lru_cache(maxsize=250_000)(self._pose_clear_quantized)

    def world_to_grid(self, x: np.ndarray, y: np.ndarray):
        # Current S09 map origin yaw is zero. Keep the transform explicit.
        dx, dy = x - self.ox, y - self.oy
        if abs(self.origin_yaw) < 1e-12:
            gx, gy = dx / self.res, dy / self.res
        else:
            c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
            gx = (c * dx + s * dy) / self.res
            gy = (-s * dx + c * dy) / self.res
        col = np.floor(gx).astype(int)
        row_from_bottom = np.floor(gy).astype(int)
        row = self.height - 1 - row_from_bottom
        return row, col

    def _pose_clear_quantized(self, ix: int, iy: int, iyaw: int) -> bool:
        # Quantized cache represents center positions at 1 cm and yaw at 1°.
        x, y = ix * 0.01 + self.ox, iy * 0.01 + self.oy
        dx, dy = self._rotated_offsets[iyaw % 360]
        wx, wy = x + dx, y + dy
        rows, cols = self.world_to_grid(wx, wy)
        if np.any(rows < 0) or np.any(rows >= self.height) or np.any(cols < 0) or np.any(cols >= self.width):
            return False
        return not bool(np.any(self.occupied[rows, cols]))

    def pose_clear(self, pose: Pose) -> bool:
        ix = math.floor((pose.x - self.ox) / 0.01 + 0.5)
        iy = math.floor((pose.y - self.oy) / 0.01 + 0.5)
        iyaw = math.floor(wrap(pose.yaw) / math.radians(1) + 0.5)
        return self._pose_clear_cached(ix, iy, iyaw)

    def swept_clear(self, a: Pose, b: Pose) -> bool:
        dyaw = wrap(b.yaw - a.yaw)
        translation = math.hypot(b.x - a.x, b.y - a.y)
        # Bound the maximum motion of a body corner, rather than checking
        # centre translation and yaw independently at a coarse interval.
        half_diag = math.hypot(self.vehicle.length/2+0.015,
                               self.vehicle.width/2+0.015)
        n = max(1, math.ceil(max(translation,abs(dyaw)*half_diag) / 0.01))
        for i in range(1, n + 1):
            t = i / n
            p = Pose(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                     wrap(a.yaw + t * dyaw))
            if not self.pose_clear(p):
                return False
        return True


def paper_asm_literal(state: Pose, ds: float, delta: float, rear_axle: float) -> Pose:
    """Literal Eq. (11) branch; retained for source-model audit, not Ranger planning."""
    return Pose(state.x + ds * math.cos(delta) * math.cos(state.yaw),
                state.y + ds * math.sin(delta) * math.sin(state.yaw),
                wrap(state.yaw - ds * math.sin(delta) / rear_axle))


def paper_asm_standardized(state: Pose, ds: float, delta: float, rear_axle: float) -> Pose:
    """Conventional bicycle interpretation, explicitly separate from literal Eq. (11)."""
    k = -math.sin(delta) / rear_axle
    return propagate(state, "A", ds, k, Vehicle(min_turn_radius=rear_axle))
