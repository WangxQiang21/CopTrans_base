"""Independent G0 acceptance probes; never edits implementation or old evidence."""
from pathlib import Path
from unittest.mock import patch
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
SUBMITTED = ROOT / 'evaluation/virtual_formation_a/G0/20261115T140936Z'
TOOL = ROOT / 'tools/virtual_a_baseline.py'
BASE = '567bdecba9676313da3bc9a97b82f57b5b569a2b'
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
           OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
spec = importlib.util.spec_from_file_location('reviewed_g0', TOOL)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
COMMANDS = []


def save(name, data):
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def run(name, args, expected):
    before = time.monotonic()
    p = subprocess.run(args, cwd=ROOT, env=ENV, text=True, capture_output=True)
    row = dict(name=name, command=args, cwd=str(ROOT), exit_code=p.returncode,
               expected_exit_code=expected, stdout=p.stdout, stderr=p.stderr,
               wall_s=time.monotonic()-before)
    COMMANDS.append(row)
    assert p.returncode == expected, row
    return p


run('fresh_cli_baseline', [sys.executable, '-B', str(TOOL), '--output',
                           str(OUT / 'fresh_baseline'), '--regression', 'baseline'], 0)
run('submitted_manifest', [sys.executable, '-B', str(TOOL), '--verify-manifest',
                           str(SUBMITTED / 'protected_after.json')], 0)
sentinel = OUT / 'negative/nonempty'
sentinel.mkdir(parents=True)
(sentinel / 'keep.txt').write_text('existing evidence must survive\n')
original = (sentinel / 'keep.txt').read_bytes()
run('nonempty_output', [sys.executable, '-B', str(TOOL), '--output', str(sentinel)], 1)
assert (sentinel / 'keep.txt').read_bytes() == original
run('historical_cli_output', [sys.executable, '-B', str(TOOL), '--output',
    str(ROOT / 'evaluation/formation_refactor_v02/FR06')], 1)

bad = json.loads((SUBMITTED / 'protected_after.json').read_text())
bad['protected_files'][0]['sha256'] = '0'*64
save('negative/bad_manifest.json', bad)
run('wrong_hash', [sys.executable, '-B', str(TOOL), '--verify-manifest',
                   str(OUT / 'negative/bad_manifest.json')], 1)
missing = json.loads((SUBMITTED / 'protected_after.json').read_text())
missing['protected_paths'].append('G0_REVIEW_NONEXISTENT_FILE')
save('negative/missing_manifest.json', missing)
run('missing_file', [sys.executable, '-B', str(TOOL), '--verify-manifest',
                     str(OUT / 'negative/missing_manifest.json')], 1)

# Missing model probe against a new fake tree; import source is the real baseline.
sys.path.insert(0, str(ROOT / 'catkin_ws/src/fwis_v3_optimizer/src'))
try:
    tool.reaudit_fr06_reference(OUT / 'negative/missing_repo')
except FileNotFoundError as exc:
    save('missing_model.json', {'passed': True, 'error': str(exc)})
else:
    raise AssertionError('missing model accepted')

# Approved future G1 setup.py edit is represented in a manifest only: no source is edited.
gate_manifest = tool.collect_manifest(ROOT, tool.DEFAULT_PROTECTED_PATHS)
gate_manifest['unknown_modifications'] = [' M catkin_ws/src/fwis_v3_optimizer/setup.py']
gate_manifest['git']['tracked_changes'] = gate_manifest['unknown_modifications']
try:
    tool._workspace_gate(ROOT, gate_manifest)
except tool.BaselineError as exc:
    gate = {'approved_G1_setup_edit_rejected': True, 'error': str(exc),
            'method': 'pure manifest simulation; actual setup.py untouched'}
else:
    gate = {'approved_G1_setup_edit_rejected': False}
save('future_stage_gate.json', gate)

# Public runner guard probe: both subprocess and write operations are mocked.
# This records would-be destinations, never writes into the frozen historical directory.
would_write = []
fake_process = subprocess.CompletedProcess([], 0, stdout='MOCK_NO_TESTS_EXECUTED', stderr='')
with patch.object(tool.subprocess, 'run', return_value=fake_process), \
        patch.object(tool, '_write_text', side_effect=lambda p, text: would_write.append(str(p))):
    rows = tool.run_regression(ROOT, ROOT / 'evaluation/formation_refactor_v02/FR06', 'geometry')
save('public_runner_guard.json', {
    'historical_output_rejected': False, 'would_write': would_write,
    'mock_records': len(rows), 'legacy_tests_executed': 0, 'historical_writes_executed': 0,
    'method': 'mocked child processes and writes; not a real regression pass'})

# Verify that actual legacy test globals resolve to each isolated case directory.
probe = """import json,runpy,sys
d=runpy.run_path(sys.argv[1],run_name='codex_redirect_probe')
print(json.dumps({k:str(d[k]) for k in ('RESULTS_PATH','ARTIFACT_DIR') if k in d}))
"""
redirects = []
for case in ('fr01_uniform_motion','fr02_a_timing','fr02_a_trajectory',
             'fr02_b_quality','fr03_uniform_search','fr04_uniform_pipeline'):
    test_file = tool.REGRESSION_CASES[case][1]
    delta = tool._redirect_environment(OUT / 'redirect_probe', case)
    args = [sys.executable,'-B','-c',probe,str(ROOT / 'tests' / test_file)]
    p = subprocess.run(args,cwd=ROOT,env=dict(ENV,**delta),text=True,capture_output=True)
    assert p.returncode == 0, (case,p.stderr)
    values = json.loads(p.stdout.strip().splitlines()[-1])
    assert values and all(value in delta.values() for value in values.values())
    redirects.append({'case':case,'resolved_globals':values,'env_delta':delta,
                      'exit_code':p.returncode,'legacy_test_cases_executed':0})
save('redirect_resolution.json', redirects)

# Independently build the original request and use the old auditor, not the submitted helper.
import numpy as np
from PIL import Image
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.uniform_quality import UniformAuditRequest, audit_uniform_timed_plan
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
fr = ROOT / 'evaluation/formation_refactor_v02/FR06'
plan = UniformTimedPlan.from_model_json((fr / 'reference/reference_model.json').read_text())
model = FormationModel((VehicleOffset(1,1,0),VehicleOffset(2,0,1),VehicleOffset(3,0,-1)))
request = UniformAuditRequest(plan_id=plan.plan_id,formation_id=plan.formation_id,
    model=model,vehicle_shapes={i:VehicleShape(.738,.5) for i in (1,2,3)},
    start_qf=FormationPose(6,6,0),start_headings={i:0 for i in (1,2,3)},
    goal_qf=FormationPose(10,10,math.pi/2),
    goal_headings={1:math.pi/2+math.atan2(1,2),2:math.pi/2,3:math.pi/2},
    timing_limits=FormationTimingLimits(),allow_reverse=True)
with Image.open(fr / 'maps/corridor_w2.8/map.pgm') as image:
    occ = np.flipud(np.asarray(image)<127).copy()
started = time.monotonic()
evaluation = audit_uniform_timed_plan(request,plan,plan.sample(),Grid(occ,.05,0,0))
assert evaluation.feasible and evaluation.status == 'FEASIBLE'
save('independent_reference.json', {'status':evaluation.status,'checks':evaluation.checks,
     'duration_s':plan.duration_s,'model_sha256':plan.model_sha256(),
     'wall_s':time.monotonic()-started,'request_from_review_constants':True,
     'submitted_reaudit_helper_used':False,'search_executed':False})

# Native Git blobs establish the initial baseline beyond self-reported before/after hashes.
bad_blobs=[];count=0
for row in subprocess.check_output(['git','ls-tree','-r','-z',BASE],cwd=ROOT).split(b'\0'):
    if not row:continue
    meta,name=row.split(b'\t',1);mode,kind,oid=meta.split();assert kind==b'blob'
    p=ROOT/name.decode();data=os.readlink(p).encode() if mode==b'120000' else p.read_bytes()
    actual=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if actual!=oid.decode():bad_blobs.append(name.decode())
    count+=1
assert not bad_blobs
save('git_blob_verification.json',{'checked':count,'mismatches':bad_blobs})

backup=Path('/home/wq/whut_thesis/_virtual_a_baseline_protection/20261115T134438Z')
originals=[]
for name in ('expv3_multi','expv3_multi_v02'):
    old=json.loads((backup/(name+'_before.json')).read_text());root=Path(old['root']);bad=[]
    for name2,entry in old['files'].items():
        p=root/name2;data=os.readlink(p).encode() if p.is_symlink() else p.read_bytes()
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:bad.append(name2)
    status=subprocess.check_output(['git','-C',str(root),'status','--porcelain=v1','--untracked-files=all'],text=True)
    head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    branch=subprocess.check_output(['git','-C',str(root),'branch','--show-current'],text=True).strip()
    assert not bad and status==old['status'] and head==old['head'] and branch==old['branch']
    originals.append({'root':str(root),'checked':len(old['files']),'changed':bad,'status_unchanged':True})
save('original_worktrees.json',originals)
snapshot=json.loads((OUT/'submission_before.json').read_text())
drift=[]
for name,h in snapshot.items():
    if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=h:drift.append(name)
assert not drift
save('submission_unchanged.json',{'checked':len(snapshot),'changed':drift})
save('commands.json',COMMANDS)
summary={'baseline_checks':'PASS','independent_reference':'FEASIBLE',
         'source_and_originals_unchanged':True,'submitted_hash_drift':drift,
         'future_G1_gate_conflict':gate['approved_G1_setup_edit_rejected'],
         'public_runner_missing_output_guard':bool(would_write),
         'old_test_cases_executed':0,'ros_started':False,'G1_started':False}
save('verification_summary.json',summary)
print(json.dumps(summary,ensure_ascii=False,indent=2))
