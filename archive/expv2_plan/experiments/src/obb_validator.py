"""Independent full-body OBB validator for spatial planning experiments.

Unlike ``RasterWorld.pose_clear``, this checker does not sample points inside
the body.  It tests overlap between the oriented vehicle rectangle and every
occupied map cell in its bounding box using separating axes.  It is therefore
used as the acceptance gate for paths produced by search or optimisation.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from spatial_model import Pose, Vehicle, wrap


class OBBRasterValidator:
    def __init__(self, map_yaml: str, vehicle: Vehicle, safety_margin: float = 0.04):
        meta = yaml.safe_load(Path(map_yaml).read_text())
        image = np.asarray(Image.open(Path(map_yaml).parent / meta["image"]).convert("L"))
        negate = bool(meta.get("negate", 0))
        probability = image.astype(float) / 255.0 if negate else (255.0-image) / 255.0
        occupied_threshold = float(meta.get("occupied_thresh", 0.65))
        free_threshold = float(meta.get("free_thresh", 0.196))
        # Unknown map cells are obstacles for a planning acceptance check.
        self.occupied = probability >= free_threshold
        self.definitely_occupied = probability > occupied_threshold
        self.res = float(meta["resolution"])
        self.ox, self.oy, self.origin_yaw = map(float, meta["origin"])
        self.height, self.width = self.occupied.shape
        self.vehicle = vehicle
        # Match the legacy validator convention: margin increases total body
        # dimensions, so each half extent receives margin/2.
        self.half_length = 0.5 * (vehicle.length + safety_margin)
        self.half_width = 0.5 * (vehicle.width + safety_margin)

    def _pose_in_map_frame(self, pose: Pose):
        dx, dy = pose.x-self.ox, pose.y-self.oy
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return c*dx+s*dy, -s*dx+c*dy, wrap(pose.yaw-self.origin_yaw)

    def pose_clear(self, pose: Pose) -> bool:
        x, y, yaw = self._pose_in_map_frame(pose)
        c, s = math.cos(yaw), math.sin(yaw)
        ux, uy = c, s
        vx, vy = -s, c
        reach_x = self.half_length*abs(ux)+self.half_width*abs(vx)
        reach_y = self.half_length*abs(uy)+self.half_width*abs(vy)
        if x-reach_x < 0 or y-reach_y < 0 or x+reach_x > self.width*self.res or y+reach_y > self.height*self.res:
            return False
        ix0 = max(0, int(math.floor((x-reach_x)/self.res)))
        ix1 = min(self.width-1, int(math.floor((x+reach_x)/self.res)))
        iy0 = max(0, int(math.floor((y-reach_y)/self.res)))
        iy1 = min(self.height-1, int(math.floor((y+reach_y)/self.res)))
        half_cell = 0.5*self.res
        for iy in range(iy0,iy1+1):
            row = self.height-1-iy
            cols = np.flatnonzero(self.occupied[row,ix0:ix1+1])+ix0
            cy = (iy+0.5)*self.res
            for ix in cols:
                cx = (ix+0.5)*self.res
                dx, dy = cx-x, cy-y
                # OBB axes.
                if abs(dx*ux+dy*uy) > self.half_length+half_cell*(abs(ux)+abs(uy)):
                    continue
                if abs(dx*vx+dy*vy) > self.half_width+half_cell*(abs(vx)+abs(vy)):
                    continue
                # Occupied-cell AABB axes.
                if abs(dx) > half_cell+self.half_length*abs(ux)+self.half_width*abs(vx):
                    continue
                if abs(dy) > half_cell+self.half_length*abs(uy)+self.half_width*abs(vy):
                    continue
                return False
        return True

    def swept_clear(self, a: Pose, b: Pose, max_corner_step: float = 0.01) -> bool:
        translation = math.hypot(b.x-a.x,b.y-a.y)
        dyaw = wrap(b.yaw-a.yaw)
        half_diag = math.hypot(self.half_length,self.half_width)
        count = max(1,math.ceil(max(translation,abs(dyaw)*half_diag)/max_corner_step))
        for i in range(count+1):
            t=i/count
            pose=Pose(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.yaw+t*dyaw))
            if not self.pose_clear(pose):
                return False
        return True

    def path_clear(self, poses: list[Pose], max_corner_step: float = 0.01):
        if not poses:
            return False, -1
        if not self.pose_clear(poses[0]):
            return False, 0
        for i,(a,b) in enumerate(zip(poses,poses[1:]),start=1):
            if not self.swept_clear(a,b,max_corner_step):
                return False,i
        return True,-1
