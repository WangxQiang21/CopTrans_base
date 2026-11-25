#!/usr/bin/env python3
"""Create a blind native request on the exact raster used by a local Gazebo experiment."""
import argparse
import gzip
from io import BytesIO
import math
from pathlib import Path
import sys
import numpy as np
from PIL import Image
import yaml
from cases import CARGO, CONTRACT, limits, packed

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("archive",type=Path);a.add_argument("--task",choices=["single","formation"],required=True)
    a.add_argument("--goal",type=float,nargs=3);a.add_argument("--profile",default="standard",choices=["standard","baseline"])
    args=a.parse_args()
    c=yaml.safe_load((args.archive/"resolved_config.yaml").read_text())
    g=yaml.safe_load((args.archive/"scene/map.yaml").read_text())
    image=Image.open(BytesIO(gzip.decompress((args.archive/"scene/map.pgm.gz").read_bytes())))
    pixels=(np.asarray(image)[::-1,:]/255)
    cells=(1-pixels if not g["negate"] else pixels)>=g["free_thresh"]
    edges=np.diff(np.r_[0,cells.ravel(),0].astype(int))
    starts=np.nonzero(edges==1)[0];ends=np.nonzero(edges==-1)[0]
    multi=args.task=="formation";margin=.05 if multi else .04
    dims=[CONTRACT[k]+2*margin for k in ("body_length_m","body_width_m")]
    def boundary(q,free):
        return dict(reference=q,heading=[q[2]]*3,constrain_reference_yaw=True,constrain_heading=[not free]*3)
    r=dict(grid=dict(width=image.width,height=image.height,resolution=float(np.float32(g["resolution"])),
                     origin=g["origin"][:2],occupied_runs=np.c_[starts,ends-starts].tolist()),
           limits=limits(args.task,args.profile),allow_reverse=True,task_duration_cap=300.,
           vehicles=[[*v,*dims] for v in CONTRACT["offsets"]] if multi else [[0,0,*dims]],
           cargo=[*CARGO["center_offset"],CARGO["size"][0]+2*margin,CARGO["size"][1]+2*margin] if multi else None,
           start=boundary(c["start"],False),goal=boundary(args.goal or c["goal"],multi))
    sys.stdout.buffer.write(packed(dict(operation="solve",request=r,budget_s=12 if multi else 8))+b"\n")
if __name__=="__main__":main()
