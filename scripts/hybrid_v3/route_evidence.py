"""Bind closed-loop results to the measured map, endpoints and odometry in each archive."""
import gzip
import hashlib
import json
import math
from statistics import median
from collections import Counter

import yaml

HISTORICAL = {
    'history_original': '20261125_115519_smXjHv',
    'history_west': '20261125_120655_6xtU9x',
    'history_heading': '20261125_004431_1tfhhn',
    'history_outdoor': '20261125_015146_OS7Yaz',
}
RELEASE_FAMILIES = (*HISTORICAL, 'long', 'multibend', 'necessary_aux', 'fault')


def digest(value):
    return hashlib.sha256(json.dumps(value, separators=(',', ':'), sort_keys=True,
                                     allow_nan=False).encode()).hexdigest()


def load_yaml(path):
    return yaml.safe_load(path.read_bytes()) if path.is_file() else {}


def pose(value):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        return None
    coords = [float(x) for x in value]
    return coords if all(math.isfinite(x) for x in coords) else None


def motion_record(path, task):
    """Sampled reference arc length, with the full sample span and endpoint coverage."""
    previous = first = None
    previous_stamp = first_stamp = None
    total = 0.
    gaps = []
    mode_sequences = {}
    used = 0
    consistent = True
    if path.is_file():
        with gzip.open(path, 'rt') as stream:
            for line in stream:
                row = json.loads(line)
                for vehicle, reported in (row.get('mode_feedback') or {}).items():
                    if reported in (0,1,2):
                        sequence=mode_sequences.setdefault(vehicle,[])
                        if not sequence or sequence[-1]!='AOS'[reported]:
                            sequence.append('AOS'[reported])
                point = pose(row.get('cargo_reference') if task == 'formation' else row.get('poses', {}).get('1'))
                stamp = row.get('stamp_s')
                if point is None or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                    consistent = False
                    continue
                if previous is not None:
                    if stamp < previous_stamp or (stamp == previous_stamp and previous != point):
                        consistent = False
                    total += math.dist(previous[:2], point[:2])
                    gaps.append(stamp - previous_stamp)
                else:
                    first, first_stamp = point, stamp
                used += 1
                previous, previous_stamp = point, stamp
    return dict(samples=used, first_pose=first, last_pose=previous, arc_m=total if used else None,
                span_sim_s=previous_stamp-first_stamp if used else None, max_gap_sim_s=max(gaps,default=None),
                nominal_period_s=median(gaps) if gaps else None,
                sampled_feedback_modes=mode_sequences, sampled_feedback_switches={k:len(v)-1 for k,v in mode_sequences.items()},
                ordered=consistent and used>0)


def read_route(folder):
    cfg = load_yaml(folder/'resolved_config.yaml')
    manifest = load_yaml(folder/'experiment.json')
    result = load_yaml(folder/'result.yaml')
    selected = load_yaml(folder/'selected_endpoints.yaml')
    scene = load_yaml(folder/'scene/scene_meta.yaml')
    grid = load_yaml(folder/'scene/map.yaml')
    pgm = folder/'scene/map.pgm.gz'
    map_sha = hashlib.sha256(gzip.decompress(pgm.read_bytes())).hexdigest() if pgm.is_file() else None
    plant = load_yaml(folder/'scene/body_twist_parameters.yaml').get('plant', {})
    task = cfg.get('scene_task')
    start, goal = pose(result.get('start')), pose(result.get('goal'))
    eligible = selected or cfg
    verified = bool(start and goal and map_sha and grid and scene and task in ('single','formation')
                    and manifest.get('experiment_id') == folder.name
                    and hashlib.sha256((folder/'scene/scene_meta.yaml').read_bytes()).hexdigest() == manifest.get('scene_sha256')
                    and map_sha == manifest.get('map_sha256') and result.get('map_matches_generated') is True
                    and start == pose(eligible.get('start')) and goal == pose(eligible.get('goal')))
    if task == 'formation':
        geometry = {k:scene.get(k) for k in ('cargo','formation_offsets_m')}
    else:
        geometry = {}
    geometry.update(frame=scene.get('frame'), obstacles=scene.get('environment_obstacles'))
    signature = dict(task=task, profile=cfg.get('speed_profile'), map_sha256=map_sha,
                     grid={k:v for k,v in grid.items() if k!='image'}, scene_geometry_sha256=digest(geometry),
                     start=start, goal=goal, controller_limits=cfg.get('controller_limit_overrides'),
                     plant_limits=plant)
    motion = motion_record(folder/'metrics/motion.jsonl.gz', task)
    chord = math.dist(start[:2],goal[:2]) if start and goal else None
    vs = (result.get('metrics') or {}).get('vehicles',{})
    raw_files = ('result.yaml','experiment.json','resolved_config.yaml','selected_endpoints.yaml',
                 'scene/map.pgm.gz','scene/map.yaml','scene/body_twist_parameters.yaml','scene/scene_meta.yaml','metrics/motion.jsonl.gz',
                 'timing_trace.jsonl','planner_parameters.yaml','controller_parameters.yaml')
    raw_files+=tuple(n for n in (manifest.get('model_artifact'),manifest.get('plan_artifact')) if n)
    return dict(run_id=folder.name, status=result.get('status'), reason=result.get('reason'),
                backend=cfg.get('planner_backend'), model_sha256=manifest.get('model_sha256'), signature=signature, signature_sha256=digest(signature),
                exact_archive=verified, chord_m=chord, motion=motion, planner_wall_s=result.get('planner_wall_s'),
                plan_duration_s=result.get('plan_duration_s'), terminal_m=max((v['terminal_position_error_m'] for v in vs.values()
                    if v.get('terminal_position_error_m') is not None),default=None),
                mount_m=(result.get('metrics') or {}).get('max_mount_error_m'),
                source_hashes={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in raw_files if (folder/n).is_file()})


def measured_completion(route, tolerance=.10):
    if not route or not route.get('exact_archive'):
        return False
    m = route['motion']; s = route['signature']
    if not m['ordered'] or m['samples'] < 2 or m['arc_m'] is None:
        return False
    # Telemetry is decimated before the final arrival. Account for the measured
    # gaps between the sampled endpoints and the authoritative requested poses.
    first_gap = math.dist(m['first_pose'][:2], s['start'][:2])
    last_gap = math.dist(m['last_pose'][:2], s['goal'][:2])
    # The frozen speed cap bounds how far the final sampled point can precede
    # the actual arrival by one typical sample interval. A large logging gap must not enlarge the tolerance.
    speed = (s.get('plant_limits') or {}).get('max_speed')
    if not isinstance(speed,(float,int)) or speed <= 0:
        return False
    coverage = tolerance + speed * m['nominal_period_s']
    return (first_gap <= tolerance and last_gap <= coverage and
            m['arc_m'] + first_gap + last_gap + 1e-6 >= route['chord_m'])


def historical_pairs(rows, reference_root):
    refs = {family:read_route(reference_root/run_id) for family,run_id in HISTORICAL.items()
            if (reference_root/run_id/'result.yaml').is_file()}
    return dict(references=refs, trials=[pair(r,refs.get(r['family'])) for r in rows if r['family'] in HISTORICAL])


def pair(row, old):
    name = row['family']; expected = HISTORICAL[name]
    rec = row.get('route') or {}; new_sig = rec.get('signature') or {}
    proof = row.get('historical') or {}; old_sig = (old or {}).get('signature') or {}
    exact_route = bool(old and rec.get('exact_archive') and old.get('exact_archive') and proof.get('source_run_id')==expected
                       and proof.get('archive_id')==expected and old.get('run_id')==expected and rec.get('run_id')==row.get('run_id')
                       and new_sig.get('task')==row['task']==proof.get('task') and new_sig.get('profile')==row['profile']==proof.get('profile')
                       and proof.get('start')==new_sig.get('start') and proof.get('goal')==new_sig.get('goal')
                       and proof.get('map_sha256')==proof.get('map_pgm_sha256')==new_sig.get('map_sha256')
                       and proof.get('archive_scene_meta_sha256')==(old.get('source_hashes') or {}).get('scene/scene_meta.yaml')
                       and bool(proof.get('generated_world_sha256'))
                       and all(new_sig.get(k)==old_sig.get(k) for k in ('map_sha256','grid','start','goal')))
    equal = bool(exact_route and rec['signature_sha256']==old['signature_sha256']
                 and all(proof.get(k) for k in ('same_task','same_profile','source_spawn_matches',
                                                'old_timing_matches','controller_overrides_match','plant_overrides_match')))
    return dict(family=name, run_id=row['run_id'], source_run_id=expected, task=row['task'], profile=row['profile'],
                trial=row['trial'], route_matches_archive=exact_route, same_closed_loop_conditions=equal,
                source_status=(old or {}).get('status'), status=row['status'], reason=row.get('reason'),
                chord_m=rec.get('chord_m'), observed_m=(rec.get('motion') or {}).get('arc_m'),
                terminal_m=rec.get('terminal_m'), mount_m=rec.get('mount_m'), planning_s=rec.get('planner_wall_s'),
                duration_s=rec.get('plan_duration_s'), authority_request_sha256=row.get('authority_request_sha256'),
                modes=row.get('motion_modes'), planned_mode_switches=row.get('planned_mode_switches'),
                sampled_feedback_switches=(rec.get('motion') or {}).get('sampled_feedback_switches',{}))


def matrix(rows, gui):
    """Explicit triplicates prevent a replacement success from hiding a failed trial."""
    if gui:
        actual = Counter((r['task'],r['profile'],r['family'],r['trial']) for r in rows)
        expected = Counter((task,profile,family,i) for task in ('single','formation') for profile in ('standard','baseline')
                           for family in RELEASE_FAMILIES for i in range(3))
    else:
        actual = Counter((r['task'],r['profile'],r['trial']) for r in rows)
        expected = Counter((task,profile,0) for task in ('single','formation') for profile in ('standard','baseline'))
        if any(r['family'] not in (*HISTORICAL,'long','multibend') for r in rows):
            return False
    return bool(actual==expected and all(r['gui']==gui for r in rows)
                and len({r['run_id'] for r in rows})==len(rows))


def route_is_bound(row):
    route=row.get('route') or {}
    files=route.get('source_hashes') or {}
    signature=route.get('signature') or {}
    required={'result.yaml','resolved_config.yaml','scene/map.pgm.gz','scene/map.yaml','scene/scene_meta.yaml',
              'scene/body_twist_parameters.yaml','experiment.json'}
    return bool(route.get('exact_archive') and required<=files.keys()
                and route.get('signature_sha256')==digest(signature)
                and signature.get('map_sha256')==row.get('map_sha256')
                and signature.get('start')==row.get('start') and signature.get('goal')==row.get('goal')
                and signature.get('task')==row['task'] and signature.get('profile')==row['profile']
                and route['run_id']==row['run_id'] and route['status']==row['status']
                and files['result.yaml']==row.get('result_sha256') and files['resolved_config.yaml']==row.get('config_sha256')
                and files['scene/scene_meta.yaml']==row.get('scene_sha256') and row.get('evidence_files',{}).get('metadata.json'))


def historical_offline(path):
    if not path.is_file():
        return {}
    summary=json.loads(path.read_bytes())
    rows=[]
    for r in summary['cases']:
        stem=path.parent/r['archive_stem']
        files={kind:stem.with_suffix('.'+kind+'.json') for kind in ('request','result','audit','ros_contract')}
        req,res,audit=(json.loads(files[k].read_bytes()) for k in ('request','result','audit'))
        if (digest(req['request'])!=r['request_sha256'] or digest(json.loads(files['ros_contract'].read_bytes()))!=r.get('ros_contract_sha256') or res.get('status')!=r['status'] or
                res.get('actual_backend')!=r.get('actual_backend') or audit!=r['audit'] or
                bool(res.get('status')=='CERTIFIED_POLICY_COMPLIANT' and audit.get('executable') is True and
                     res.get('actual_backend')==r['expected_backend'])!=r['passed']):
            raise ValueError('Changed historical offline evidence: '+str(stem))
        proof=r['proof']
        rows.append({**{k:r.get(k) for k in ('archive_id','task','profile','request_sha256','ros_contract_sha256','map_sha256','start','goal','passed',
                                              'status','reason','modes','solve_wall_s','duration_s','reference_length_m','plan_sha256')},
                     'source_plan':proof.get('source_plan'), 'source_status':proof.get('source_status'),
                     'source_task':proof.get('source_task'),'source_profile':proof.get('source_profile'),
                     'raw_sha256':{k:hashlib.sha256(f.read_bytes()).hexdigest() for k,f in files.items()}})
    return {**{k:summary.get(k) for k in ('source_commit','core_commit','binary_sha256','library_sha256')},
            'summary_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'cases':rows}
