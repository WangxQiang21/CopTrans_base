import csv
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
import yaml

from clock_receipts import PROGRAMS, audit, vehicle


def write_csv(path, rows):
    with gzip.open(path, 'wt', newline='') as out:
        writer=csv.DictWriter(out,fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def fixture(tmp_path):
    name='20261125_100000_demo'
    directory=tmp_path/'campaign_1'/name
    archive_root=tmp_path/'runs'
    archive=archive_root/name
    directory.mkdir(parents=True)
    archive.mkdir(parents=True)
    model=json.dumps({'epoch_ns':900_000_000}).encode()
    digest=hashlib.sha256(model).hexdigest()
    (archive/'model.gz').write_bytes(gzip.compress(model))
    commit='f'*40
    manifest={'experiment_id':name,'status':'passed','git_commit':commit,'model_sha256':digest,'model_artifact':'model.gz'}
    (archive/'experiment.json').write_text(json.dumps(manifest))
    result=dict(experiment_id=name,status='passed',reason='complete',speed_profile='standard',task='single',
                model_sha256=digest,routed_plan_keys={1:[3,900_000_000]})
    (archive/'result.yaml').write_text(yaml.safe_dump(result))
    (archive/'timing_trace.jsonl').write_text(json.dumps(dict(event='plan_received',run_id=name,model_sha256=digest,
                                                             plan_id=3,planner_ros_stamp_ns=900_000_000))+'\n')
    installed={}
    for program in sorted(PROGRAMS | {'libcoptrans_bridge_take_trace.so'}):
        path=tmp_path/program
        shutil.copyfile('/bin/true',path)
        if program in PROGRAMS:
            installed[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    m=dict(run_id=name,family='long',trial=0,profile='standard',task='single',gui=True,source_commit=commit,
           status='passed',reason='complete',archive='reports/runs/'+name,installed_binaries=installed)
    (directory/'metadata.json').write_text(json.dumps(m))
    controls=[];plant=[];bridge=[]
    for i in range(2):
        clock=1_000_000_000+i*40_000_000
        wall=7_000_000_000+i*40_000_000
        command=.125+i*.1
        t=dict(timer_period_ns=40_000_000,timer_due_clock_ns=clock,timer_called_clock_ns=clock,
               callback_ended_clock_ns=clock,timer_called_steady_ns=wall,callback_started_steady_ns=wall+5,
               command_publish_steady_ns=wall+10,command_published_steady_ns=wall+20,callback_ended_steady_ns=wall+30,
               odom_stamp_ns=clock-10_000_000,odom_callback_steady_ns=wall-5_000_000,
               odom_source_realtime_ns=1000+i,odom_received_realtime_ns=1020+i,status_drops=0)
        controls.append(dict(plan_id=3,trajectory_stamp_ns=900_000_000,model_sha256=digest,
                             command=[command,0.,0.],stamp=clock/1e9,phase='TRACKING',control_timing=t))
        row=dict(event='command',identity=i+1,observed_steady_ns=wall+47,consume_steady_ns=wall+52,
                 applied_steady_ns=wall+55,vx_mps=command,vy_mps=0.,wz_radps=0.)
        plant.extend([dict(row,event='odometry',identity=t['odom_stamp_ns'],observed_steady_ns=wall-6_000_000),row])
        bridge.append(dict(robot='robot1',observed_steady_ns=wall+40,taken_realtime_ns=4020+i,
                           source_realtime_ns=4010+i,received_realtime_ns=4018+i,vx_mps=command,
                           vy_mps=0.,wz_radps=0.,trace_overflow=0))
    with gzip.open(directory/'robot1_controller.jsonl.gz','wt') as h:
        for r in controls:h.write(json.dumps(r)+'\n')
    write_csv(directory/'robot1_plant_timing.csv.gz',plant)
    write_csv(directory/'bridge_take_123.csv.gz',bridge)
    return directory,archive_root,controls,plant,bridge


def test_raw_clocks_and_tampered_or_missing_evidence(tmp_path):
    run,archive_root,_,_,_=fixture(tmp_path)
    entry=audit(run,True,archive_root)
    clock=entry['vehicles']['robot1']
    assert entry['flags']==[] and clock['rtf']==1 and clock['causal_bridge_to_plant']['n']==2
    first=tmp_path/'first.json'
    verify=tmp_path/'verified.json'
    cmd=[sys.executable,str(Path(__file__).with_name('clock_receipts.py')),'--gazebo',str(run),
         '--archive-root',str(archive_root),'--output']
    subprocess.run(cmd+[str(first)],check=True)
    subprocess.run(cmd+[str(verify),'--verify',str(first)],check=True)
    assert first.read_bytes()==verify.read_bytes()
    # Even semantically identical raw rows with changed bytes must fail an existing signature.
    meta=run/'metadata.json'
    meta.write_bytes(meta.read_bytes()+b'\n')
    result=subprocess.run(cmd+[str(verify),'--verify',str(first)],capture_output=True,text=True)
    assert result.returncode==1 and 'fingerprint changed' in result.stderr
    (run/'bridge_take_123.csv.gz').unlink()
    with pytest.raises(ValueError,match='missing or unexpected DDS'):
        audit(run,True,archive_root)


def test_skips_overrun_wall_stale_and_identity(tmp_path):
    _,_,controlled,plant,bridge=fixture(tmp_path)
    identity=(3,900_000_000,controlled[0]['model_sha256'])
    t=controlled[-1]['control_timing']
    t['timer_called_clock_ns']+=40_000_000
    t['callback_ended_clock_ns']+=80_000_001
    t['callback_ended_steady_ns']+=6_000_000
    controlled[-1]['stamp']+=.30
    out=vehicle(controlled,plant,bridge,'robot1',identity)
    assert out['ros_misses']==2 and out['stale_motion']==1
    assert {'ros_deadline_miss','stale_motion','callback_over_5ms'}<=set(out['flags'])
    controlled[-1]['trajectory_stamp_ns']+=1
    with pytest.raises(ValueError,match='trajectory identity'):
        vehicle(controlled,plant,bridge,'robot1',identity)


@pytest.mark.parametrize('resume', [False, True], ids=['fresh-trial', 'resumed-trial'])
def test_monitored_matrix_stops_before_next_trial_on_a_physical_pass(tmp_path, monkeypatch, resume):
    # Importing the driver is read-only. Only the native archive and control
    # timing are real inputs here; the simulator launch itself is replaced.
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2]/'ros2_ws/src/fwis_bringup'))
    import closed_loop
    run,archive_root,controls,_,_=fixture(tmp_path)
    (tmp_path/'reports').mkdir()
    (tmp_path/'reports/runs').symlink_to(archive_root,target_is_directory=True)
    controls[-1]['control_timing']['callback_ended_steady_ns']+=5_100_000
    control_file=run/'robot1_controller.jsonl.gz'
    with gzip.open(control_file,'wt') as h:
        for row in controls:h.write(json.dumps(row)+'\n')
    raw_before=control_file.read_bytes()
    meta_file=run/'metadata.json'
    meta_before=meta_file.read_bytes()
    launches=[]
    def simulated_run(args,family,task,trial):
        launches.append(trial)
        return {'status':'passed','experiment_id':run.name}
    monkeypatch.setattr(closed_loop,'ROOT',tmp_path)
    monkeypatch.setattr(closed_loop,'supervise',simulated_run)
    monkeypatch.setattr(closed_loop,'fixture_config',lambda *args: {})
    monkeypatch.setattr(closed_loop,'same_fixture',lambda *args: True)
    monkeypatch.setattr(closed_loop,'binaries',lambda:json.loads(meta_before)['installed_binaries'])
    cli=['closed_loop','--families','long','--task','single','--profile','standard','--repetitions','2',
         '--stop-on-failure','--output',str(run.parent)]
    monkeypatch.setattr(sys,'argv',cli+(['--resume'] if resume else []))
    with pytest.raises(SystemExit,match=run.name+'.*robot1 callback_over_5ms'):
        closed_loop.main()
    assert launches==([] if resume else [0])
    assert control_file.read_bytes()==raw_before and meta_file.read_bytes()==meta_before
