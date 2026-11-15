"""FR02-A decoder time-domain probes; no S9/dynamic feasibility audit."""
import copy
import json
from pathlib import Path
import sys
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'tests').is_dir() and (p/'catkin_ws').is_dir())
sys.path.insert(0,str(ROOT/'tests'))
import test_uniform_trajectory as f
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
OUT=Path(__file__).resolve().parent
plan=f.spin_plan()
results=[]
for case in ('subtolerance_gap','unrepresentable_motion_interval'):
    model=copy.deepcopy(plan.to_model_dict())
    phase=model['phases'][1]
    if case=='subtolerance_gap':
        phase['t0_s']+=5e-10
        phase['duration_s']-=5e-10
        at=(model['phases'][0]['end_time_s']+phase['t0_s'])/2
    else:
        phase['duration_s']=1e-20
        at=phase['t0_s']
    phase['end_time_s']=phase['t0_s']+phase['duration_s']
    model['duration_s']=phase['end_time_s']
    (OUT/(case+'_input.json')).write_text(json.dumps(model,indent=2)+'\n')
    record={'case':case,'expected':'REJECT_INVALID_TIME_STRUCTURE','time':at,'motion_t0':phase['t0_s'],'motion_duration':phase['duration_s'],'motion_end':phase['end_time_s']}
    try:
        restored=UniformTimedPlan.from_model_dict(model)
    except Exception as e:
        record.update(decoder='REJECTED',reason=getattr(e,'reason',type(e).__name__),status='PASSED')
    else:
        record['decoder']='ACCEPTED'
        try:
            state=restored.state_at(at)
        except Exception as e:
            record.update(status='FAILED',actual='LEGAL_DOMAIN_TIME_HAS_NO_STATE',reason=getattr(e,'reason',type(e).__name__),detail=str(e))
        else:
            record.update(status='FAILED',actual='COLLAPSED_PHASE_RETURNS_WRONG_ENDPOINT',terminal_progress=state.progress,expected_progress=restored.edges[-1].progress_interval[1])
    results.append(record)
(OUT/'decoder_time_probes.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
sys.exit(1 if any(r['status']=='FAILED' for r in results) else 0)
