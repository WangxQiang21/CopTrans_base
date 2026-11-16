"""Independent R1/R2 recheck. No implementation changes or legacy TestCase runs."""
from pathlib import Path
from unittest.mock import patch
import copy
import hashlib
import importlib.util
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
TOOL=ROOT/'tools/virtual_a_baseline.py'
spec=importlib.util.spec_from_file_location('codex_g0_rework',TOOL)
va=importlib.util.module_from_spec(spec)
spec.loader.exec_module(va)
P='catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/'
ENV=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')


def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(data,f,ensure_ascii=False,indent=2);f.write('\n')


base=va.collect_manifest(ROOT,va.DEFAULT_PROTECTED_PATHS)
expected_files={
 'G1':[P+n for n in ['__init__.py','contracts.py','geometry.py','bounds.py']]+[
  'tests/test_virtual_a_geometry.py','tests/test_virtual_a_profile.py','tools/virtual_a_benchmark.py'],
 'G2':[P+'optimizer.py',P+'audit.py','tests/test_virtual_a_optimizer.py',
  'tests/test_virtual_a_audit.py','tests/fixtures/virtual_a/open_requests.json'],
 'G3':[P+'trajectory.py',P+'timing.py','tests/test_virtual_a_timing.py','tests/test_virtual_a_trajectory.py'],
 'G4':[P+'collision.py',P+'pipeline.py','tools/virtual_a_scenes.py',
  'tests/test_virtual_a_collision.py','tests/test_virtual_a_pipeline.py','tests/fixtures/virtual_a/maps/review.pgm'],
 'G5':[P+'ros_adapter.py','tests/test_virtual_a_ros_adapter.py',
  'tests/test_virtual_a_playback.py','tests/ros_virtual_a_smoke.py'],
}
matrix=[]
for stage,paths in expected_files.items():
    level='geometry' if stage=='G1' else 'timing' if stage in ('G2','G3') else 'full'
    for path in paths:
        manifest=copy.deepcopy(base)
        manifest['git']['tracked_changes']=[]
        manifest['untracked']['unknown']=[path]
        manifest['unknown_modifications']=[]
        try:va._workspace_gate(ROOT,manifest,level=level);actual='ACCEPT';detail=''
        except va.BaselineError as e:actual='REJECT';detail=str(e)
        matrix.append({'task_stage':stage,'regression_level':level,'path':path,
                       'expected':'ACCEPT','actual':actual,'detail':detail})
save('task_path_matrix.json',matrix)
unresolved=[r for r in matrix if r['actual']!=r['expected']]
assert [r['path'] for r in unresolved]==['tests/fixtures/virtual_a/open_requests.json']

gate_cases=[]
for level,status,accepted in [
 ('geometry',' M catkin_ws/src/fwis_v3_optimizer/setup.py',True),
 ('baseline',' M catkin_ws/src/fwis_v3_optimizer/setup.py',False),
 ('full',' M catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml',False),
 ('full',' M catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation.py',False),
 ('full',' D catkin_ws/src/fwis_v3_optimizer/setup.py',False),
 ('full',' M catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py',True),
 ('timing',' M catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py',False)]:
    manifest=copy.deepcopy(base);manifest['git']['tracked_changes']=[status]
    try:va._workspace_gate(ROOT,manifest,level=level);actual=True;detail=''
    except va.BaselineError as e:actual=False;detail=str(e)
    assert actual==accepted,(level,status,detail)
    gate_cases.append({'level':level,'status':status,'expected_accept':accepted,'accepted':actual,'detail':detail})
save('gate_positive_negative.json',gate_cases)

# R2: path and overwrite failures must precede both subprocess and write.
fixture=OUT/'runner_fixtures';fixture.mkdir()
existing_artifact=fixture/'artifact';(existing_artifact/'regression_artifacts/geometry_continuous').mkdir(parents=True)
existing_log=fixture/'log';(existing_log/'logs').mkdir(parents=True)
(existing_log/'logs/geometry_formation.stderr.log').write_text('KEEP\n')
second_case=fixture/'second';(second_case/'regression_artifacts/geometry_formation').mkdir(parents=True)
metadata=fixture/'metadata';metadata.mkdir();(metadata/'protected_before.json').write_text('{}\n')
historical=ROOT/'evaluation/formation_refactor_v02/FR06'
symlink=fixture/'history_link';symlink.symlink_to(historical,target_is_directory=True)
cases=[('history',historical,False),('external',Path('/tmp/g0_recheck_external'),False),
 ('root',ROOT/'evaluation/virtual_formation_a',False),('artifact',existing_artifact,False),
 ('log',existing_log,False),('second_case_preflight',second_case,False),
 ('symlink',symlink,False),('metadata_allowed',metadata,True),
 ('clean',fixture/'clean',True)]
runner=[]
for name,target,expected in cases:
    writes=[];fake=subprocess.CompletedProcess([],0,stdout='MOCK_NO_TESTS_EXECUTED',stderr='')
    with patch.object(va.subprocess,'run',return_value=fake) as children, \
         patch.object(va,'_write_text',side_effect=lambda p,s:writes.append(str(p))):
        try:result=va.run_regression(ROOT,target,'geometry');actual=True;detail=''
        except va.BaselineError as e:actual=False;detail=str(e)
        count=children.call_count
    assert actual==expected,(name,detail)
    if not expected:assert count==0 and not writes,(name,count,writes)
    else:assert count==2 and all(OUT in Path(p).parents for p in writes)
    runner.append({'case':name,'expected_accept':expected,'accepted':actual,
                   'mock_child_calls':count,'mock_writes':writes,'detail':detail,
                   'real_old_tests':0,'real_historical_writes':0})
assert (existing_log/'logs/geometry_formation.stderr.log').read_text()=='KEEP\n'
save('independent_runner_guards.json',runner)

commands=[]
def run(name,args,expected):
    p=subprocess.run(args,cwd=ROOT,env=ENV,text=True,capture_output=True)
    record={'name':name,'command':args,'cwd':str(ROOT),'exit_code':p.returncode,
            'expected_exit_code':expected,'stdout':p.stdout,'stderr':p.stderr}
    commands.append(record);assert p.returncode==expected,record
run('fresh_baseline',[sys.executable,'-B',str(TOOL),'--output',str(OUT/'fresh_baseline'),'--regression','baseline'],0)
run('repeat_refused',[sys.executable,'-B',str(TOOL),'--output',str(OUT/'fresh_baseline'),'--regression','baseline'],1)
run('manifest',[sys.executable,'-B',str(TOOL),'--verify-manifest',str(OUT/'fresh_baseline/protected_after.json')],0)
save('commands.json',commands)

# Check baseline bytes against native Git objects and both original worktrees.
bad=[];count=0
for row in subprocess.check_output(['git','ls-tree','-r','-z',va.BASELINE_HEAD],cwd=ROOT).split(b'\0'):
    if not row:continue
    meta,name=row.split(b'\t',1);mode,kind,oid=meta.split();assert kind==b'blob'
    p=ROOT/name.decode();data=os.readlink(p).encode() if mode==b'120000' else p.read_bytes()
    digest=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if digest!=oid.decode():bad.append(name.decode())
    count+=1
assert not bad
save('baseline_blobs.json',{'checked':count,'changed':bad})
backup=Path('/home/wq/whut_thesis/_virtual_a_baseline_protection/20261115T134438Z');originals=[]
for name in ['expv3_multi','expv3_multi_v02']:
    old=json.loads((backup/(name+'_before.json')).read_text());root=Path(old['root']);drift=[]
    for rel,entry in old['files'].items():
        p=root/rel;data=os.readlink(p).encode() if p.is_symlink() else p.read_bytes()
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:drift.append(rel)
    actual_status=subprocess.check_output(['git','-C',str(root),'status','--porcelain=v1','--untracked-files=all'],text=True)
    assert not drift and actual_status==old['status']
    assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()==old['head']
    originals.append({'root':str(root),'checked':len(old['files']),'changed':drift,'git_status_unchanged':True})
save('original_worktrees.json',originals)
submitted=json.loads((OUT/'submission_before.json').read_text())
drift=[rel for rel,h in submitted.items() if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=h]
assert not drift
save('submitted_evidence_unchanged.json',{'checked':len(submitted),'changed':drift})
assert not (ROOT/'catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a').exists()
summary={'R2':'CLOSED','R1':'PARTIALLY_FIXED_ONE_MANDATORY_FIXTURE_MISSING',
         'authorized_task_paths_checked':len(matrix),'unresolved_paths':[r['path'] for r in unresolved],
         'runner_cases_passed':len(runner),'baseline_blobs_verified':count,
         'submission_drift':drift,'old_test_cases_executed':0,'ROS_started':False,'G1_started':False}
save('verification_summary.json',summary)
print(json.dumps(summary,ensure_ascii=False,indent=2))
