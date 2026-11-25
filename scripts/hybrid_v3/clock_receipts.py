#!/usr/bin/env python3
"""Reconstruct per-run simulation timing from tracker, native DDS and Gazebo plant records."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import re
import sys
from collections import Counter, defaultdict
from functools import lru_cache
from pathlib import Path

import yaml

from cases import ROOT
sys.path.append(str(ROOT / 'ros2_ws/src/fwis_control/test'))
from analyze_control_timing import pair_commands

PERIOD = 40_000_000
FRESH = 200_000_000
PROGRAMS = {'hybrid_joint_planner', 'hybrid_joint_executor', 'ranger_tracking_controller_cpp',
            'libfwis_hybrid_joint.so', 'libhybrid_codec.so'}


def require(condition, error):
    if not condition:
        raise ValueError(error)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    ordered = sorted(values)
    return dict(n=len(ordered), p99_ms=ordered[math.ceil(len(ordered)*.99)-1]*1.e-6,
                max_ms=ordered[-1]*1.e-6) if ordered else {'n': 0, 'p99_ms': None, 'max_ms': None}


def gzip_rows(path, csv_format):
    with gzip.open(path, 'rt', newline='') as inp:
        return list(csv.DictReader(inp) if csv_format else map(json.loads, inp))


@lru_cache(maxsize=128)
def elf_digest(path, size, mtime_ns, ctime_ns, inode):
    with path.open('rb') as inp:
        require(inp.read(4) == b'\x7fELF', f'Not a native ELF: {path}')
    return sha(path)


def binaries(record, system):
    got = {}
    probe = None
    for name, expected in record.items():
        path = Path(name)
        if system:
            relative = path.parts[path.parts.index('install')+1:]
            path = system/'ros2_ws/install'/Path(*relative)
        st = path.stat()
        actual = elf_digest(path, st.st_size, st.st_mtime_ns, st.st_ctime_ns, st.st_ino)
        require(actual == expected, f'Native artifact changed: {name}')
        require(path.name not in got, f'Duplicate native artifact: {path.name}')
        got[path.name] = actual
        if path.name=='ranger_tracking_controller_cpp':
            probe_path=path.with_name('libcoptrans_bridge_take_trace.so')
            ps=probe_path.stat()
            probe=elf_digest(probe_path,ps.st_size,ps.st_mtime_ns,ps.st_ctime_ns,ps.st_ino)
    require(set(got) == PROGRAMS, 'Incomplete bounded set of native artifacts')
    return got,probe


def vehicle(controllers, plant, bridge, robot, identity):
    all_timed = [r for r in controllers if r.get('control_timing') and 'command' in r]
    tracking = [r for r in all_timed if r['phase'] == 'TRACKING']
    require(tracking, f'{robot} missing active native callbacks')
    require(all('control_timing' in r and 'command' in r for r in controllers
                if r.get('phase') == 'TRACKING'), f'{robot} missing timing on tracking record')
    flags = set()
    skipped = overruns = stale = missing_slots = missing_odom = dropped_status = 0
    callback_ns, publish_ns, odom_transport = [], [], []
    sources = {int(r['identity']):int(r['observed_steady_ns']) for r in plant if r['event'] == 'odometry'}
    require(sources, f'{robot} missing plant odometry')
    counted_stamps = set()
    previous = None
    for r in all_timed:
        if r['phase'] != 'TRACKING':
            previous = None
            continue
        t = r['control_timing']
        require((r['plan_id'],r['trajectory_stamp_ns'],r['model_sha256']) == identity,
                f'{robot} tracker and physical run have different trajectory identity')
        values = r['command']
        require(len(values)==3 and all(math.isfinite(v) for v in values), f'{robot} invalid native command')
        due, called, end_ros = (t[k] for k in ('timer_due_clock_ns','timer_called_clock_ns','callback_ended_clock_ns'))
        start, publish, submitted, end = (t[k] for k in ('callback_started_steady_ns','command_publish_steady_ns',
                                                        'command_published_steady_ns','callback_ended_steady_ns'))
        require(t['timer_period_ns']==PERIOD and 0 < due <= called <= end_ros and
                0 < t['timer_called_steady_ns'] <= start <= publish <= submitted <= end,
                f'{robot} invalid clock domain, period or callback ordering')
        require(publish not in counted_stamps, f'{robot} duplicate callback')
        counted_stamps.add(publish)
        missed = max(0,called-due)//PERIOD
        skipped += missed
        overruns += int(end_ros > due+(missed+1)*PERIOD)
        if previous:
            previous_due, previous_misses, previous_start = previous
            require(due>previous_due and start>previous_start, f'{robot} non-monotone releases')
            expected = previous_due+(previous_misses+1)*PERIOD
            delta = due-expected
            if delta != 0:
                flags.add('tracking_release_discontinuity')
                missing_slots += max(0,delta)//PERIOD
        previous = (due,missed,start)
        feedback = t['odom_stamp_ns']
        ros_age = round(r['stamp']*1e9)-feedback
        wall_age = start-t['odom_callback_steady_ns']
        # A Gazebo odometry callback can arrive ahead of the latest /clock tick.
        # Only positive age exceeding either documented 200 ms cutoff is stale.
        stale += int(any(values) and (ros_age>FRESH or wall_age>FRESH))
        if feedback not in sources:
            missing_odom += 1
        else:
            require(sources[feedback]<=t['odom_callback_steady_ns']<=start,
                    f'{robot} plant odometry and control clocks differ')
        emitted, received = t['odom_source_realtime_ns'], t['odom_received_realtime_ns']
        if 0<emitted<=received:
            odom_transport.append(received-emitted)
        else:
            flags.add('invalid_dds_odometry_source_stamp')
        dropped_status=max(dropped_status,t['status_drops'])
        callback_ns.append(end-start)
        publish_ns.append(submitted-publish)
    applied = [r for r in plant if r['event']=='command']
    require(applied and bridge, f'{robot} missing DDS bridge or plant command records')
    require(all(int(r['observed_steady_ns'])<=int(r['consume_steady_ns'])<=int(r['applied_steady_ns'])
                for r in applied), f'{robot} invalid plant command timing')
    require(all(int(r['trace_overflow'])==0 for r in bridge), f'{robot} DDS capture overflow')
    received, unmatched_plant = pair_commands(all_timed,applied)
    taken, unmatched_bridge = pair_commands(all_timed,bridge)
    takes = {when:r for when,r in taken if when in counted_stamps}
    end_to_end, dds_transport, bridge_delivery = [], [], []
    for when,r in received:
        if when not in counted_stamps:
            continue
        observed = int(r['observed_steady_ns'])
        end_to_end.append(int(r['consume_steady_ns'])-when)
        take = takes.get(when)
        if take is None:
            continue
        midpoint = int(take['observed_steady_ns'])
        if when <= midpoint <= observed:
            sent=int(take['source_realtime_ns'])
            got=int(take['received_realtime_ns'])
            if 0<sent<=got:
                dds_transport.append(got-sent)
            bridge_delivery.append(observed-midpoint)
    first,last=tracking[0],tracking[-1]
    if len(tracking)==1:
        idx=next(i for i,r in enumerate(all_timed) if r is first)
        first,last=all_timed[max(0,idx-1)],all_timed[min(len(all_timed)-1,idx+1)]
    start_t,end_t=first['control_timing'],last['control_timing']
    wall=end_t['callback_started_steady_ns']-start_t['callback_started_steady_ns']
    sim=end_t['timer_called_clock_ns']-start_t['timer_called_clock_ns']
    if wall<=0 or sim<=0:flags.add('missing_rtf_window')
    if skipped or overruns:flags.add('ros_deadline_miss')
    if stale:flags.add('stale_motion')
    if missing_odom:flags.add('unmatched_odometry_identity')
    if dropped_status:flags.add('native_status_overflow')
    if max(callback_ns)>5_000_000:flags.add('callback_over_5ms')
    if not end_to_end:flags.add('no_unique_plant_delivery')
    if not bridge_delivery:flags.add('no_unique_controller_bridge_plant_delivery')
    if len(dds_transport)!=len(bridge_delivery):flags.add('invalid_dds_command_source_stamp')
    return dict(robot=robot, tracking_samples=len(tracking), total_timed_callbacks=len(all_timed),
                plant_odom_samples=len(sources), plant_command_samples=len(applied), bridge_samples=len(bridge),
                ros_misses=skipped+overruns, skipped_ros_releases=skipped, overrun_ros_callbacks=overruns,
                missing_release_slots=missing_slots, stale_motion=stale, uncorrelated_odom_ticks=missing_odom,
                status_drops=dropped_status, wall_s=wall/1e9, sim_s=sim/1e9, rtf=sim/wall if wall else None,
                rtf_window='tracking' if len(tracking)>1 else 'adjacent_callbacks',
                callback=stats(callback_ns), publication=stats(publish_ns), publish_to_plant=stats(end_to_end),
                causal_bridge_to_plant=stats(bridge_delivery), dds_command=stats(dds_transport),
                dds_odometry=stats(odom_transport), plant_matching=unmatched_plant, bridge_matching=unmatched_bridge,
                flags=sorted(flags))


def audit(directory, gui, archive_root, system=None):
    run_id=directory.name
    require(bool(re.fullmatch(r'[0-9]{8}_[0-9]{6}_[A-Za-z0-9]+',run_id)),f'Invalid run ID: {run_id}')
    meta_file=directory/'metadata.json'
    meta=json.loads(meta_file.read_bytes())
    require(meta['run_id']==run_id and meta['gui'] is gui and meta['task'] in ('single','formation'),
            f'{run_id} metadata scope mismatch')
    require(Path(meta['archive'])==Path('reports/runs')/run_id, f'{run_id} wrong archive identity')
    require(not meta.get('bridge_capture_error'),f'{run_id} bridge capture explicitly failed')
    archive=archive_root/run_id
    result_path,manifest_path,trace_path=(archive/n for n in ('result.yaml','experiment.json','timing_trace.jsonl'))
    out=yaml.safe_load(result_path.read_bytes())
    manifest=json.loads(manifest_path.read_bytes())
    events=[json.loads(ln) for ln in trace_path.read_bytes().splitlines() if ln]
    require(all(e.get('run_id')==run_id for e in events),f'{run_id} timing trace identity mismatch')
    model_path=archive/manifest['model_artifact']
    model_blob=gzip.decompress(model_path.read_bytes())
    model_sha=hashlib.sha256(model_blob).hexdigest()
    model=json.loads(model_blob)
    require(out['experiment_id']==manifest['experiment_id']==run_id and
            out['status']==manifest['status']==meta['status'] and out['reason']==meta['reason'] and
            out['speed_profile']==meta['profile'] and out['task']==('multi' if meta['task']=='formation' else 'single'),
            f'{run_id} runner/archive status, profile or task mismatch')
    require(model_sha==manifest['model_sha256']==out['model_sha256'],f'{run_id} authoritative model fingerprint mismatch')
    require(meta['source_commit']==manifest['git_commit'],f'{run_id} frozen source mismatch')
    if system:
        require((system/'source_commit').read_text().strip()==meta['source_commit'],f'{run_id} requested system mismatch')
    received=[e for e in events if e['event']=='plan_received' and e.get('model_sha256')==model_sha]
    require(len(received)==1,f'{run_id} missing unique plan publication')
    identity=(received[0]['plan_id'],model['epoch_ns'],model_sha)
    require(received[0]['planner_ros_stamp_ns']==identity[1],f'{run_id} inconsistent plan epoch')
    hashes={f'archive/{p.relative_to(archive)}':sha(p) for p in (result_path,manifest_path,trace_path,model_path)}
    hashes['metadata.json']=sha(meta_file)
    require(not list(directory.glob('*.journal')),f'{run_id} unfinalized bridge journal')
    robots=['robot1'] if meta['task']=='single' else ['robot1','robot2','robot3']
    groups=defaultdict(list)
    for p in sorted(directory.glob('bridge_take_*.csv.gz')):
        hashes[p.name]=sha(p)
        for row in gzip_rows(p,True):
            groups[row['robot']].append(row)
    require(set(groups)==set(robots),f'{run_id} missing or unexpected DDS bridge traces; present {sorted(groups)}')
    if 'bridge_take_samples' in meta:
        require(meta['bridge_take_samples']=={k:len(v) for k,v in groups.items()},f'{run_id} bridge sample inventory mismatch')
    require({p.name.split('_')[0] for p in directory.glob('robot*_controller.jsonl.gz')}==set(robots) and
            {p.name.split('_')[0] for p in directory.glob('robot*_plant_timing.csv.gz')}==set(robots),
            f'{run_id} missing or extra tracker/plant evidence')
    measurements={}
    for i,robot in enumerate(robots,1):
        keys=out['routed_plan_keys']
        require(tuple(keys[i] if i in keys else keys[str(i)])==identity[:2],f'{run_id} {robot} routed plan identity mismatch')
        ctl=directory/(robot+'_controller.jsonl.gz');plant=directory/(robot+'_plant_timing.csv.gz')
        hashes.update({p.name:sha(p) for p in (ctl,plant)})
        measurements[robot]=vehicle(gzip_rows(ctl,False),gzip_rows(plant,True),groups[robot],robot,identity)
    flags=sorted({flag for row in measurements.values() for flag in row['flags']})
    if out['status']!=('stopped' if meta['family']=='fault' else 'passed'):flags.append('run_outcome')
    installed,probe=binaries(meta['installed_binaries'],system)
    return dict(id=run_id,**{k:meta[k] for k in ('status','gui','family','trial','profile','task','source_commit')},
                plan_sha256=model_sha, flags=flags, ros_misses=sum(r['ros_misses'] for r in measurements.values()),
                stale_motion=sum(r['stale_motion'] for r in measurements.values()), vehicles=measurements,
                evidence_sha256=hashes, installed_elf_sha256=installed, resolved_bridge_probe_elf_sha256=probe)


def generate(campaigns, archive_root, system=None):
    reports={}
    for scope,gui in campaigns:
        scope=scope.resolve()
        runs=[scope] if (scope/'metadata.json').is_file() else sorted(p for p in scope.iterdir() if p.is_dir())
        require(runs,f'No trials in {scope}')
        for directory in runs:
            key=directory.parent.name+'/'+directory.name
            require(key not in reports,f'Duplicate run: {key}')
            reports[key]=audit(directory,gui,archive_root,system)
    groups=Counter((key.split('/')[0],r['gui']) for key,r in reports.items())
    for key,row in reports.items():
        row['coverage']={'audited_runs_in_input_campaign':groups[(key.split('/')[0],row['gui'])],
                         'required_release_runs_in_scope':96 if row['gui'] else 4}
    return reports


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--gazebo',action='append',type=Path,default=[],help='GUI campaign directory or one finalized GUI run')
    cli.add_argument('--headless',action='append',type=Path,default=[],help='headless campaign directory or one finalized headless run')
    cli.add_argument('--archive-root',type=Path,default=ROOT/'reports/runs')
    cli.add_argument('--system',type=Path,help='rebind native artifact hashes against a relocated frozen system snapshot')
    cli.add_argument('--output',type=Path,required=True)
    cli.add_argument('--verify',type=Path,help='compare the entire reconstructed index to the original receipt')
    args=cli.parse_args()
    scopes=[(path,gui) for paths,gui in ((args.gazebo,True),(args.headless,False)) for path in paths]
    if not scopes:cli.error('Provide --gazebo or --headless')
    if args.verify and args.verify.resolve()==args.output.resolve():cli.error('Verification output must differ from its source')
    try:
        rows=generate(scopes,args.archive_root,args.system)
        if args.verify:
            require(rows==json.loads(args.verify.read_bytes()),'Clock receipt or fingerprint changed')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(rows,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
        failed={key:r['flags'] for key,r in rows.items() if r['flags']}
        if failed:
            cli.exit(1, f'Independent clock checks failed: {failed}\n')
        print(f'{len(rows)} independently audited simulation runs')
    except (EOFError, OSError, KeyError, ValueError, TypeError, ZeroDivisionError) as exc:
        cli.exit(1,f'Clock evidence rejected: {exc}\n')

if __name__=='__main__':
    main()
