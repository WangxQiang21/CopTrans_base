#!/usr/bin/env python3
"""Independently audit collected published results and open secondary review."""
import argparse
import hashlib
import gzip
import json
import math
from pathlib import Path
import sys
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'catkin_ws/src/fwis_v3_optimizer/src'))
from fwis_v3_optimizer.collection import CollectionStore, identity, atomic_json, serve, utc


def analyze(store):
    import numpy as np
    if not hasattr(np, 'trapz'):
        np.trapz = np.trapezoid
    from fwis_v3_optimizer.core import Pose, Action, Grid
    from fwis_v3_optimizer.vehicle_execution import restore_plan, vehicle_contract
    from fwis_v3_optimizer.retiming import MotionLimits, TimedPlan
    from fwis_v3_optimizer.continuous_retiming import ContinuousTimedPlan
    from fwis_v3_optimizer.quality import TrajectoryEvaluator, QualityProfile
    analyzer_files = [Path(__file__)] + [ROOT/'catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer'/name
        for name in ('core.py','quality.py','continuous_geometry.py','continuous_retiming.py','vehicle_execution.py')]
    analyzer_hash = identity({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in analyzer_files})
    results = []
    for row in store.load():
        result = {'schema':'coptrans_collected_analysis_v1','record_id':row['record_id'],
                  'result_sha256':row['result_sha256'],'analyzed_at':utc(),'analyzer_source_sha256':analyzer_hash,
                  'eligible':False,'flags':[],'reason':None}
        try:
            content = {k:row.get(k) for k in ('request','map_id','execution_contract','quality_report','timed_trace','corridors','outcome')}
            if identity(content) != row['result_sha256']:
                raise ValueError('保存的结果哈希不符')
            if row.get('map_changed_during_request'):
                raise ValueError('规划中地图发生变化，不能作为可复现示范')
            if not row.get('final') or not row.get('timed_trace'):
                raise ValueError('没有完成的已发布轨迹：'+str(row.get('outcome')))
            c=row['execution_contract'];limits=MotionLimits(**c['limits'])
            expected=vehicle_contract(c['length_m'],c['width_m'],c['margin_total_dimension_m'],limits)
            if c != expected:
                raise ValueError('车辆配置身份不符')
            with gzip.open(str(store.root/'maps'/(row['map_id']+'.json.gz')),'rt') as f:
                m=json.load(f)
            if identity(m) != row['map_id']:
                raise ValueError('地图像素或坐标哈希不符')
            occ=np.asarray(m['data'],dtype=np.int16).reshape(m['height'],m['width'])
            grid=Grid((occ>=50)|(occ<0),m['resolution'],*m['origin'],c['length_m'],c['width_m'],c['margin_total_dimension_m'])
            request=row['request'];start,goal=Pose(*request['start']),Pose(*request['goal'])
            session_path=Path(row['record_path']).parents[2]/'session.json'
            session=json.loads(session_path.read_text())
            if session['execution_contract'] != c:
                raise ValueError('会话车辆配置不符')
            profile=QualityProfile(**session['quality_profile'])
            report=row['quality_report'];direction=report.get('direction_contract',{})
            reverse=direction.get('allow_reverse',True);shunting=direction.get('allow_shunting',False)
            evaluator=TrajectoryEvaluator(grid,start,goal,limits,c['length_m'],c['width_m'],.02,math.pi/180,profile,allow_reverse=reverse,allow_shunting=shunting)
            trace=[(float(t),Pose(x,y,yaw),float(v),int(mode)) for t,x,y,yaw,v,mode in row['timed_trace']]
            timing=report['selected_timing'];blocks=report.get('selected_continuous_plan')
            if blocks:
                plan=restore_plan(start,goal,blocks)
                timed=ContinuousTimedPlan(trace,timing['phases'],timing['duration_s'],timing['translation_s'],timing['spin_s'],timing['mode_switch_hold_seconds'],timing['mode_switch_count'],timing['same_mode_hold_seconds'],timing['gear_change_hold_seconds'],limits,
                    timing.get('speed_constraint_intervals'),timing.get('low_speed_intervals'),timing['interpolation_version'])
                audit=evaluator.evaluate_continuous_geometry(plan,timed,allow_reverse=reverse)
            else:
                actions=[Action(int(mode),amount,control) for mode,amount,control in report['selected_actions']]
                timed=TimedPlan(trace,timing['phases'],timing['duration_s'],timing['translation_s'],timing['spin_s'],timing['reconfiguration_s'],timing['reconfiguration_count'],limits)
                audit=evaluator.evaluate(actions,timed,allow_reverse=reverse)
            display_clear=all(evaluator.sweep_grid.interpolated_clear(a[1],b[1],.005) for a,b in zip(trace,trace[1:]))
            result.update(audit=audit,display_sweep_clear=display_clear,
                          eligible=audit.get('status')=='FEASIBLE' and display_clear,
                          execution_s=timing['duration_s'],switches=timing['mode_switch_count'],
                          reason=audit.get('reason') if display_clear else '发布插值扫掠碰撞')
            if not result['eligible']:
                result['flags'].append('共同复核未通过')
            if row.get('label')=='approved' and not result['eligible']:
                result['flags'].append('初筛优质但复核失败')
        except (ValueError,KeyError,OSError,ArithmeticError) as e:
            result['reason']=str(e);result['flags'].append('没有可用的完整审核证据')
        results.append((row,result))
    groups={};duplicates={}
    for row,a in results:
        if not a['eligible']:continue
        group=(row['task_id'],row['execution_contract']['sha256'])
        groups.setdefault(group,[]).append((row,a))
        key=(group,identity(row['timed_trace']))
        if key in duplicates:a['flags'].append('定时轨迹与 '+duplicates[key]+' 完全重复')
        else:duplicates[key]=row['record_id']
    for values in groups.values():
        fastest=min(values,key=lambda v:v[1]['execution_s'])
        for row,a in values:
            if a['execution_s']>fastest[1]['execution_s']*1.02:
                a['flags'].append('相同任务及车辆配置下，比 '+fastest[0]['record_id']+' 慢 %.2f s'%(a['execution_s']-fastest[1]['execution_s']))
    for row,a in results:
        atomic_json(Path(row['record_path']).parent/'analysis.json',a)
        atomic_json(Path(row['record_path']).parent/'analysis_summary.json',
                    {k:a.get(k) for k in ('result_sha256','eligible','flags','execution_s','switches','reason')})
    print(json.dumps({'records':len(results),'independently_eligible':sum(a['eligible'] for _,a in results),'human_labels_unchanged':True},ensure_ascii=False),flush=True)


def export_reviewed(store,path):
    approved=[];comparison=[]
    for r in store.load():
        a=r.get('analysis',{})
        ref={k:r.get(k) for k in ('record_id','record_path','result_sha256','task_id','backend','label','notes','review_label','review_notes')}
        ref['raw_file_sha256'] = hashlib.sha256(Path(r['record_path']).read_bytes()).hexdigest()
        ref['analysis_file_sha256'] = hashlib.sha256((Path(r['record_path']).parent/'analysis.json').read_bytes()).hexdigest() if a else None
        if r.get('review_label')=='approved':
            if not a.get('eligible'):
                raise ValueError('二次优质标签缺少有效复核：'+r['record_id'])
            ref['reference_fields']=['quality_report.selected_continuous_plan','quality_report.selected_actions','quality_report.selected_timing','timed_trace']
            approved.append(ref)
        elif r.get('review_label')=='comparison':comparison.append(ref)
    if path.exists():raise FileExistsError(path)
    atomic_json(path,{'schema':'coptrans_reviewed_collection_v1','created_at':utc(),'actor_candidates':approved,'comparison_candidates':comparison,'training_started':False})
    print('已导出二次人工筛选清单：',path,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root',type=Path,default=ROOT.parent/'expv3_experiment_archive/teacher_collection')
    parser.add_argument('--reanalyze',action='store_true',help='重新审核实际保存的几何和定时轨迹，不修复或改写结果')
    parser.add_argument('--audit-only',action='store_true')
    parser.add_argument('--export-reviewed',type=Path)
    parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--no-browser',action='store_true')
    args=parser.parse_args();store=CollectionStore(args.data_root,secondary=True)
    if args.reanalyze or args.audit_only:analyze(store)
    if args.export_reviewed:export_reviewed(store,args.export_reviewed)
    if args.audit_only or args.export_reviewed:return
    server,url=serve(store,ROOT/'tools/collection_panel.html',args.port)
    print('二次筛选页面：'+url,flush=True)
    if not args.no_browser:webbrowser.open(url)
    try:
        while True:time.sleep(.5)
    except KeyboardInterrupt:server.shutdown()


if __name__=='__main__':main()
