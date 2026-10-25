"""Feature-disc safe travel corridors built around a coarse pose path."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from spatial_model import Pose, Vehicle
from obb_validator import OBBRasterValidator


@dataclass(frozen=True)
class FeatureDisc:
    longitudinal_offset: float
    lateral_offset: float
    radius: float


@dataclass(frozen=True)
class CorridorBox:
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def contains(self,x:float,y:float,tol:float=1e-9):
        return self.xmin-tol <= x <= self.xmax+tol and self.ymin-tol <= y <= self.ymax+tol


def feature_disc_cover(vehicle:Vehicle,longitudinal_count:int=4,lateral_count:int=2,
                       safety_margin:float=0.04):
    """Return a disc grid whose union covers the expanded body rectangle."""
    if longitudinal_count < 1 or lateral_count < 1:
        raise ValueError("feature disc grid counts must be positive")
    length=vehicle.length+safety_margin
    width=vehicle.width+safety_margin
    dx=length/longitudinal_count
    dy=width/lateral_count
    radius=math.hypot(0.5*dx,0.5*dy)
    return [FeatureDisc(-0.5*length+(i+0.5)*dx,
                        -0.5*width+(j+0.5)*dy,radius)
            for i in range(longitudinal_count) for j in range(lateral_count)]


class SafeCorridorBuilder:
    """Build axis-aligned free boxes for each feature disc at each pose.

    Occupancy is eroded by each disc radius before box growth.  Consequently,
    constraining the disc centre to its box keeps that entire disc outside raw
    occupied cells at optimisation nodes.  A continuous OBB sweep remains the
    final acceptance gate between nodes.
    """
    def __init__(self,validator:OBBRasterValidator,features:list[FeatureDisc],max_extent:float=1.0):
        if abs(validator.origin_yaw)>1e-10:
            raise NotImplementedError("corridor builder currently requires a zero-yaw map origin")
        self.map=validator
        self.features=features
        self.max_extent=max_extent
        # Padding makes the outside of the map an obstacle in the distance field.
        free=~validator.occupied
        padded=np.pad(free,1,constant_values=False)
        self.clearance=distance_transform_edt(padded)[1:-1,1:-1]*validator.res
        self._allowed={}

    def _allowed_for(self,radius:float):
        key=round(radius,9)
        if key not in self._allowed:
            # EDT is measured centre-to-centre.  One cell diagonal covers the
            # occupied-cell area and arbitrary position inside a free cell.
            guard=math.sqrt(2.0)*self.map.res
            self._allowed[key]=self.clearance >= radius+guard
        return self._allowed[key]

    def _feature_center(self,pose:Pose,feature:FeatureDisc):
        c,s=math.cos(pose.yaw),math.sin(pose.yaw)
        return (pose.x+feature.longitudinal_offset*c-feature.lateral_offset*s,
                pose.y+feature.longitudinal_offset*s+feature.lateral_offset*c)

    def _one_box(self,x:float,y:float,radius:float):
        allowed=self._allowed_for(radius)
        ix=int(math.floor((x-self.map.ox)/self.map.res))
        iy=int(math.floor((y-self.map.oy)/self.map.res))
        if ix<0 or iy<0 or ix>=self.map.width or iy>=self.map.height:
            raise ValueError("feature centre lies outside map")
        row=self.map.height-1-iy
        if not allowed[row,ix]:
            raise ValueError("coarse path feature disc has insufficient clearance")
        left=right=ix
        bottom=top=iy
        max_cells=max(1,int(math.floor(self.max_extent/self.map.res)))
        changed=True
        while changed:
            changed=False
            candidates=("left","right","bottom","top")
            for side in candidates:
                if side=="left" and ix-left<max_cells and left>0:
                    col=left-1; rows=slice(self.map.height-1-top,self.map.height-bottom)
                    if np.all(allowed[rows,col]): left=col; changed=True
                elif side=="right" and right-ix<max_cells and right+1<self.map.width:
                    col=right+1; rows=slice(self.map.height-1-top,self.map.height-bottom)
                    if np.all(allowed[rows,col]): right=col; changed=True
                elif side=="bottom" and iy-bottom<max_cells and bottom>0:
                    newrow=self.map.height-1-(bottom-1)
                    if np.all(allowed[newrow,left:right+1]): bottom-=1; changed=True
                elif side=="top" and top-iy<max_cells and top+1<self.map.height:
                    newrow=self.map.height-1-(top+1)
                    if np.all(allowed[newrow,left:right+1]): top+=1; changed=True
        return CorridorBox(self.map.ox+left*self.map.res,
                           self.map.ox+(right+1)*self.map.res,
                           self.map.oy+bottom*self.map.res,
                           self.map.oy+(top+1)*self.map.res)

    def build(self,poses:list[Pose]):
        corridors=[]
        for pose in poses:
            row=[]
            for feature in self.features:
                x,y=self._feature_center(pose,feature)
                row.append(self._one_box(x,y,feature.radius))
            corridors.append(row)
        return corridors
