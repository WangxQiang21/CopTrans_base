"""Deterministic independent witness construction. The solver receives only the request."""
import hashlib
import json
import math
from pathlib import Path
import random

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.ndimage import distance_transform_edt
import yaml

ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / "reports/hybrid_v3"
LOCAL = ROOT / "reports/acceptance/hybrid_v3"
SPEC = REPORT / "acceptance_spec.yaml"
REGISTRY = ROOT / "ros2_ws/src/fwis_bringup/config/test_defaults.yaml"
GEOMETRY = ROOT / "ros2_ws/src/fwis_common/config/ranger_mini_2.yaml"
MASTER_SEEDS = {"core": 921117, "development": 825921, "holdout": 19312019, "pruning": 335117}
POLICY = {"weights": [0.5, 1.0, 0.25], "radius": 0.9, "budget_single": 8, "budget_formation": 12}
PROFILES = yaml.safe_load(REGISTRY.read_text())
VEHICLE = yaml.safe_load(GEOMETRY.read_text())
CONTRACT = yaml.safe_load((ROOT / "ros2_ws/src/fwis_backends/multi_4wis/config/transport_formation.yaml").read_text())
CARGO = yaml.safe_load((ROOT / "ros2_ws/src/ranger_mini_gz/config/cargo_geometry.yaml").read_text())["cargo"]


def packed(data):
    return json.dumps(data, separators=(",", ":"), allow_nan=False, sort_keys=True).encode()


def digest(data):
    return hashlib.sha256(packed(data)).hexdigest()


def limits(task, profile):
    kind = "single" if task == "single" else "multi"
    p = PROFILES["profiles"][kind]
    override = PROFILES["speed_profiles"][profile].get(kind, {})
    base = VEHICLE["quality"]["limits"]
    if kind == "single":
        vals = base | p["quality"]["limits"] | override
        dynamics = dict(speed=vals["v_max"], acceleration=vals["a_max"], jerk=vals["jerk_max"],
                        lateral_acceleration=vals["lateral_a_max"], wheel_speed=vals["wheel_speed_max"],
                        spin_speed=vals["omega_max"], spin_acceleration=vals["alpha_max"],
                        spin_jerk=vals["angular_jerk_max"], yaw_rate=vals["translation_omega_max"],
                        yaw_acceleration=vals["translation_alpha_max"], yaw_jerk=vals["translation_angular_jerk_max"])
    else:
        # FormationTimingLimits defaults, overridden by the same speed profile as launch.
        vals = p["timing_limits"] | override
        dynamics = dict(speed=vals["v_max"], acceleration=vals["a_max"], jerk=2.0,
                        lateral_acceleration=0.6, wheel_speed=1.0, spin_speed=vals["yaw_rate_max"],
                        spin_acceleration=vals["yaw_accel_max"], spin_jerk=3.0,
                        yaw_rate=vals["yaw_rate_max"], yaw_acceleration=vals["yaw_accel_max"], yaw_jerk=3.0)
    return dict(dynamics, radius=.9, wheelbase=VEHICLE["vehicle"]["wheelbase"], track=VEHICLE["vehicle"]["track"],
                steer_angle=base["steer_angle_max"], steer_rate=base["steer_rate_max"],
                steer_acceleration=base["steer_accel_max"], prepare_time=.70)


def indices(split):
    if split == "core":
        return [i for i in range(1, 13) for _ in range(3)]
    if split == "development":
        return [i for i in range(1, 13) for _ in range(5)]
    if split == "pruning":
        return [1, 2, 3, 4, 7] * 2 + [10, 10]
    # Adjacent holdout cases are matched rigid transformations of the same request.
    return ([i for i in range(1, 8) for _ in range(10)]
            + [i for i in range(8, 12) for _ in range(10)] + [12] * 10)


def phase(mode, kind, pieces, duration, first, last=None, gear=1):
    return dict(mode=mode, kind=kind, duration=duration, pieces=pieces, heading0=first,
                heading1=first if last is None else last, gear=[gear] * 3)


def stationary(at):
    return [[[x] * 6 for x in at]]


def make(split, index, task, profile):
    family = indices(split)[index]
    pair = index // 2 if split == "holdout" else index
    seed = MASTER_SEEDS[split] + pair * 1301 + (task == "formation") * 3211
    rng = random.Random(seed)
    angle = rng.uniform(-.48, .48) + ((index % 2) * 1.239 if split == "holdout" else 0)
    c, s = math.cos(angle), math.sin(angle)
    length = rng.uniform(4.5, 7.5) if family not in (5, 6) else rng.uniform(21, 25)
    sy = rng.choice([-1, 1])
    amp = sy * rng.uniform(.6, 1.4)
    x0 = rng.uniform(-1, 1) - .5 * length * c
    y0 = rng.uniform(-1, 1) - .5 * length * s
    fraction = np.linspace(0, 1, 11)
    # A quintic B-spline with simple interior knots (C4). Convert exactly on
    # each knot span into degree-five Bernstein coefficients via endpoint jets.
    if family in (2, 4, 8, 9, 11):
        y = amp * fraction**2 * (1.7 if family in (2, 4) else .6)
    elif family == 3:
        y = amp * (10*fraction**3 - 15*fraction**4 + 6*fraction**5)
    elif family in (5, 6):
        y = amp * (1.7 if family == 6 else 1.5) * np.sin(fraction * math.pi * (3 if family == 6 else 2))
    elif family == 10:
        y = length*fraction
    else:
        y = fraction*0
    x = (0*fraction if family == 10 else length*fraction) * (-1 if family == 7 else 1)
    curve = make_interp_spline(fraction, np.stack((x0 + c*x - s*y, y0 + s*x + c*y), axis=-1), k=5)
    # Uniform subdivisions ensure equal station widths even for the end spans.
    pieces = []
    for a,b in zip(fraction[:-1],fraction[1:]):
        h=b-a
        q0,q5=curve(a),curve(b)
        q1=q0+h*curve(a,1)/5
        q4=q5-h*curve(b,1)/5
        q2=q0+2*(q1-q0)+h*h*curve(a,2)/20
        q3=q5+2*(q4-q5)+h*h*curve(b,2)/20
        coordinates=np.asarray([q0,q1,q2,q3,q4,q5]).T
        pieces.append([*coordinates.tolist(), [angle]*6])
    gear = -1 if family == 7 else 1
    def heading(t):
        v = curve(t,1) * gear
        return math.atan2(v[1],v[0]) if family != 10 else angle + .02
    h0,h1=[heading(0)]*3,[heading(1)]*3
    q0=[*curve(0).tolist(),angle]; q1=[*curve(1).tolist(),angle]
    multi=task=="formation"
    geom=VEHICLE["vehicle"]
    margin=.05 if multi else .04
    length_body,width_body=geom["length"]+2*margin,geom["width"]+2*margin
    off_scale=rng.uniform(.96,1.06)
    vehicles=[[a*off_scale,b*off_scale,length_body,width_body] for a,b in CONTRACT["offsets"]] if multi else [[0,0,length_body,width_body]]
    cargo=([CARGO["center_offset"][0]+rng.uniform(-.05,.05),rng.uniform(-.05,.05),
            CARGO["size"][0]+2*margin+rng.uniform(-.10,.10),CARGO["size"][1]+2*margin+rng.uniform(-.10,.10)] if multi else None)
    lim=limits(task,profile)
    path=curve(np.linspace(0,1,1001))
    max_ds=float(np.max(np.linalg.norm(curve(np.linspace(0,1,501),1),axis=1)))
    duration=max(4.0,2.02*max_ds/lim["speed"])
    mode="O" if family==10 else "A"
    main=phase(mode,"motion",pieces,duration,h0,h1,gear)
    initial_prepare=phase("A","prepare",stationary(q0),5.,h0,gear=gear)
    witness=[initial_prepare,main]
    initial=h0[:]; terminal=h1[:]
    if family in (8,9):
        delta=[rng.uniform(.28,.48)*(1 if i%2==0 else -1) for i in range(3)]
        if family==8:
            initial=[a+b for a,b in zip(h0,delta)]
            witness=[phase("S","prepare",stationary(q0),5.,initial),
                     phase("S","motion",stationary(q0),6.,initial,h0),
                     phase("A","prepare",stationary(q0),5.,h0), main]
        else:
            terminal=[a+b for a,b in zip(h1,delta)]
            witness=[initial_prepare,main,phase("S","prepare",stationary(q1),5.,h1),
                     phase("S","motion",stationary(q1),6.,h1,terminal)]
    elif family==10:
        witness=[phase("O","prepare",stationary(q0),6.,h0),main]
    def boundary(ref,heads,free=False):
        return dict(reference=[*ref[:2],ref[2] if multi else heads[0]],heading=heads,
                    constrain_reference_yaw=True,constrain_heading=[not free]*3)
    # Rasterization defines the actual test map. Certification uses occupied
    # cells including the rerasterized boundary, not the continuous drawing.
    size=360; res=.10
    cells=np.zeros((size,size),dtype=np.uint8)
    if family in (5,6,10,11):
        centerline=np.ones_like(cells)
        xy=np.floor(path/res+size/2).astype(int)
        centerline[xy[:,1],xy[:,0]]=0
        # Keep the entire inflated load and each independently yawing car inside
        # the end caps, even after occupancy is rasterized under a rigid rotation.
        vehicle_reach=max(math.hypot(a,b)+.5*math.hypot(l,w) for a,b,l,w in vehicles)
        reach=vehicle_reach
        if cargo:
            a,b,l,w=cargo
            reach=max(reach,math.hypot(abs(a)+l/2,abs(b)+w/2))
        radius=reach+math.sqrt(2)*res+rng.uniform(.03,.22)
        cells=(distance_transform_edt(centerline)*res > radius).astype(np.uint8)
    if family==12:
        ix,iy=np.floor(curve(1)/res+size/2).astype(int)
        cells[iy-2:iy+3,ix-2:ix+3]=1
        if multi:  # Barrier intersects the goal cargo at its known reference.
            assert abs(cargo[0]) < cargo[2]/2
    flat=np.r_[0,cells.ravel(),0].astype(int)
    starts=np.nonzero(np.diff(flat)==1)[0]; ends=np.nonzero(np.diff(flat)==-1)[0]
    request=dict(grid=dict(width=size,height=size,resolution=res,origin=[-size*res/2]*2,
                           occupied_runs=np.column_stack((starts,ends-starts)).tolist()),
                 vehicles=vehicles,cargo=cargo,start=boundary(q0,initial),
                 goal=boundary(q1,terminal,multi and family!=9 and index%3==0),
                 limits=lim, allow_reverse=True, task_duration_cap=450.0)
    if split=="holdout":
        request["goal"]["constrain_heading"]=[not (multi and family!=9 and pair%3==0)]*3
    label="invalid" if family==12 else "A" if family<=7 else "other"
    return request,([] if family==12 else witness),dict(family=f"C{family:02}",label=label,seed=seed,
            rotation_pair=pair if split=="holdout" else None,rotated=bool(split=="holdout" and index%2))
