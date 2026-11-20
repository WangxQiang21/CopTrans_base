import argparse, json, math, sys
from unittest import mock
import numpy as np
sys.path.insert(0, '/home/wq/whut_thesis/expv3_multi_virtual_a/tests')
sys.path.insert(0, '/home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/src/fwis_v3_optimizer/src')
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1, GuideRoute,
    grid_signature, research_planner_config)
from fwis_v3_optimizer.formation_continuous_a import pipeline as pipeline_module
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.route_processing import simplify_route
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    build_corridor_chain, make_corridor_guide, allocate_corridors,
    audit_corridor_chain, audit_path_in_corridors)
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from virtual_a_g4_support import make_request, map_request

ap = argparse.ArgumentParser()
ap.add_argument('--a', type=float); ap.add_argument('--b', type=float)
ap.add_argument('--deg', type=float); ap.add_argument('--pad', type=float, default=3.0)
ap.add_argument('--psi0', type=float, default=0.0)
ap.add_argument('--goal-heading', default='leg')
ap.add_argument('--obstacle-frac', type=float, default=0.5)
ap.add_argument('--only', default='backward')
ap.add_argument('--samples', type=int, default=61)
ap.add_argument('--alloc-dump', action='store_true')
ap.add_argument('--block', default='')
args = ap.parse_args()

theta = math.radians(args.deg)
px, py = args.a + args.b*math.cos(theta), args.b*math.sin(theta)
mx, my = args.obstacle_frac*px, args.obstacle_frac*py
xmax = px + 4.0; ymax = py + 5.0
cols = int((xmax + 3.0)/0.05) + 10
rows = int((ymax + 4.0)/0.05) + 10
g = Grid(np.zeros((rows, cols), dtype=bool), 0.05, -3.0, -4.0, length=0.738, width=0.500)
ix = int((mx - g.ox)/g.res); iy = int((my - g.oy)/g.res)
g.occ[iy-2:iy+3, ix-2:ix+3] = True
if args.block:
    bx, by = [float(v) for v in args.block.split(',')]
    jx = int((bx - g.ox)/g.res); jy = int((by - g.oy)/g.res)
    g.occ[jy-5:jy+6, jx-5:jx+6] = True
route = GuideRoute(points=((0.,0.), (float(args.a),0.), (float(px),float(py))),
                   source='e09_prof', grid_sha256=grid_signature(g))
start = FormationPose(0.,0.,float(args.psi0))
psi_goal = theta if args.goal_heading == 'leg' else 0.0
goal = FormationPose(px, py, psi_goal)
simp = simplify_route(g, route)
chain = build_corridor_chain(g, simp, box_padding_m=args.pad)
audit = audit_corridor_chain(g, chain)
guide = make_corridor_guide(chain, (0.,0.), (px,py)) if chain.regions else None
allocation = allocate_corridors(chain, guide, 8) if guide is not None else ()
print('CHAIN', len(chain.regions), audit['passed'], audit['reason'])
print('GUIDE', [(round(q[0],3),round(q[1],3)) for q in guide.points])
print('ALLOC', [(round(x,4),round(y,4),i) for x,y,i in allocation])
print('REGIONS')
for ri, reg in enumerate(chain.regions):
    print('  R', ri, 'A', tuple(tuple(round(float(v),3) for v in row) for row in reg.A),
          'b', tuple(round(float(v),3) for v in reg.b))

captured = []
orig = optimizer_module.certify_formation_path
def wrapped(path, request, ch, template, **kwargs):
    cert = orig(path, request, ch, template, **kwargs)
    captured.append((path, cert, template, kwargs.get('allocation'), ch))
    return cert
optimizer_module.certify_formation_path = wrapped

audits = []
_orig_sample = optimizer_module._formation_sample_audit
def _sample_wrap(path, template, ch):
    audit = _orig_sample(path, template, ch)
    audits.append((path, audit, template, ch))
    return audit
optimizer_module._formation_sample_audit = _sample_wrap

import dataclasses
_orig_build = pipeline_module.build_formation_template_candidates
def _wrapped_build(*a, **kw):
    ts = _orig_build(*a, **kw)
    if args.only == 'all':
        return ts
    idx = {'allocation': 0, 'forward': 1, 'backward': 2}[args.only]
    chosen = tuple(t for i, t in enumerate(ts.templates) if i == idx)
    if not chosen:
        return ts
    return dataclasses.replace(ts, templates=chosen)
pipeline_module.build_formation_template_candidates = _wrapped_build

planner = research_planner_config(guidance_mode='corridor', corridor_enforcement='hard',
    formation_corridor_scope=FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    corridor_box_padding_m=args.pad, max_initials=6, max_seconds=60.0)
req = make_request(start, goal, start_headings={1:args.psi0,2:args.psi0,3:args.psi0})
with mock.patch.object(pipeline_module, 'make_route_guides', return_value=(route,)):
    result = plan_virtual_a(map_request(req, g, planner=planner))
fa = result.formation_corridor_audit or {}
print('RESULT', result.status, result.reason, 'exec', result.executable)
print('ATTEMPTS', [(t.get('template_rank'), t.get('geometry_status'),
                    t.get('geometry_reason'), t.get('certificate_status'),
                    t.get('accepted'), t.get('initial_count'))
                   for t in fa.get('template_attempts', [])])
print('CAPTURED', len(captured), 'SAMPLE_AUDITS', len(audits))
runs_src = []
for path, cert, template, alloc, ch in captured:
    runs_src.append((path, cert, template, alloc, ch, 'cert'))
cert_paths = {id(row[0]) for row in captured}
for path, audit, template, ch in audits:
    if id(path) in cert_paths:
        continue
    runs_src.append((path, {'status': str(audit.get('status', '')),
                            'reason': str(audit.get('reason', '')),
                            'audit': audit},
                     template, allocation, ch, 'sample'))
for ai, (path, cert, template, alloc, ch, src) in enumerate(runs_src):
    print('\n== CAPTURE', ai, src, 'cert', cert.get('status'), cert.get('reason'),
          'L', round(float(path.length_m),3),
          'margin', template.margin_m)
    if src == 'sample':
        a = cert.get('audit') or {}
        print('SAMPLE AUDIT reason', a.get('reason'),
              'first_failure', a.get('first_failure'),
              'max', a.get('max_residual_m'))
    print('SEGS', [(round(sg.sigma_start,4), round(sg.sigma_end,4),
                    int(sg.f_region_index)) for sg in template.segments])
    print('COVERS', [[(c.vehicle_id, c.decomposition,
                       [int(p.region_index) for p in c.pieces])
                      for c in sg.covers] for sg in template.segments])
    L = float(path.length_m)
    n = max(1, args.samples)
    def f_res(p, base):
        reg = ch.regions[int(base)]
        return max(float(row[0])*p[0] + float(row[1])*p[1] - float(lim)
                   for row, lim in zip(reg.A, reg.b))
    def alloc_index(sig):
        for a, b, i in alloc:
            if float(a) - 1e-9 <= sig < float(b) - 1e-12:
                return int(i)
            if sig >= float(b) - 1e-12 and abs(float(b)-1.0) < 1e-12:
                return int(i)
        return int(alloc[-1][2]) if alloc else None
    def seg_f(sig):
        for sg in template.segments:
            if float(sg.sigma_start) - 1e-9 <= sig <= float(sg.sigma_end) + 1e-9:
                return int(sg.f_region_index)
        return int(template.segments[-1].f_region_index)
    worst_alloc = None; worst_tmpl = None
    print('PROFILE sigma  Fx      Fy     psi     alloc tmpl  r_alloc  r_tmpl(+m)  %s')
    for k in range(n+1):
        sig = k/n
        st = path.state_at_s(min(L, max(0.0, sig*L)))
        f = (float(st.qf.x), float(st.qf.y))
        ai_ = alloc_index(sig); tf = seg_f(sig)
        ra = f_res(f, ai_)
        rt = f_res(f, tf)
        rtm = rt + float(template.margin_m)
        if worst_alloc is None or ra > worst_alloc[0]:
            worst_alloc = (ra, sig, f)
        if worst_tmpl is None or rtm > worst_tmpl[0]:
            worst_tmpl = (rtm, sig, f)
        mark = '*' if ai_ != tf else ' '
        if ai_ != tf or k % max(1, n//24) == 0:
            print('  {} {:.4f} {:7.3f} {:7.3f} {:6.3f}  {:3d}  {:3d}  {:+.3e} {:+.3e}'
                  .format(mark, sig, f[0], f[1],
                          float(st.qf.psi), ai_, tf, ra, rtm))
    print('WORST r_alloc (nominal alloc region) {:.3e} at sigma {:.4f} F {}'
          .format(worst_alloc[0], worst_alloc[1],
                  tuple(round(v,3) for v in worst_alloc[2])))
    print('WORST r_tmpl+margin {:.3e} at sigma {:.4f}'.
          format(worst_tmpl[0], worst_tmpl[1]))
    if alloc and ch is not None:
        old = audit_path_in_corridors(
            path, ch, alloc,
            position_budget_m=float(planner.corridor_audit_position_margin_m),
            max_depth=int(planner.corridor_audit_max_depth),
            max_intervals=int(planner.corridor_audit_max_intervals),
            min_width_m=float(planner.corridor_audit_min_width_m))
        ff = old.get('first_failure') or {}
        print('OLD_AUDIT passed', old.get('passed'), old.get('reason'),
              'max_bound', old.get('max_bound_residual_m'),
              'ff', {k: ff.get(k) for k in ('s_m', 'sigma', 'corridor_index',
                                            'reason', 'point_residual_m',
                                            'bound_residual_m', 'width_m')}
              if ff else None)
    if cert.get('unclosed_intervals'):
        u = cert['unclosed_intervals'][0]
        print('FIRST UNCLOSED', round(u.get('s0_m',0),4), u.get('open_count'),
              [(o.get('kind'), o.get('region_index'), o.get('vehicle_id'),
                o.get('halfspace_id'), '{:.3e}'.format(o.get('upper_m') or 0.0))
               for o in (u.get('open') or [])[:4]])