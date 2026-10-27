"""Compare recorded offline frontend runs and draw their certified trajectories."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from .benchmark import engineering_tasks
from .configuration import load_settings
from .dataset import artifact_directory
from .kernel_adapter import PROJECT_ROOT
from .training_figures import draw_problem


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation',action='append',required=True,help='label=directory')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = artifact_directory(args.output)
    settings = load_settings()
    problems,inputs = {},[]
    for problem,entry in engineering_tasks(settings.contract):
        problems[problem.task_id] = problem
        inputs.append(entry)
    reports,manifests,files = {},{},{}
    for entry in args.evaluation:
        label,path = entry.split('=',1)
        if label in reports: raise ValueError('duplicate_label')
        directory = Path(path)
        raw = directory/'evaluation.json'
        report = json.loads(raw.read_text())
        rows = {row['task_id']:row for row in report['tasks']}
        if set(rows)!=set(problems): raise ValueError('engineering_task_set_mismatch')
        manifest = json.loads((directory/'manifest.json').read_text())
        if manifest['identity']['contract']!=settings.contract.identity:
            raise ValueError('vehicle_contract_mismatch')
        for name,row in rows.items():
            problem = problems[name]
            for key in ('start','goal'):
                if not all(math.isclose(row[key][k],v,abs_tol=1e-10)
                           for k,v in asdict(getattr(problem,key)).items()):
                    raise ValueError('task_pose_mismatch:'+name)
            if row['is_success']:
                if not all(row['finalization']['audit'].get(k)=='PASS' for k in
                    ('geometry_status','dynamic_status','timing_status')):
                    raise ValueError('success_without_certificate')
        reports[label] = rows
        manifests[label] = manifest
        files[label] = {'path':str(raw),'sha256':hashlib.sha256(raw.read_bytes()).hexdigest()}
    configs = [m['config'] for m in manifests.values()]
    if any(c!=configs[0] for c in configs):raise ValueError('frontend_budget_mismatch')
    labels = list(reports)
    common = [name for name in problems if all(reports[label][name]['is_success'] for label in labels)]
    summaries = {}
    for label,rows in reports.items():
        summaries[label] = {
            'tasks':len(rows),'successes':sum(r['is_success'] for r in rows.values()),
            'mean_all_wall_s':float(np.mean([r['wall_s'] for r in rows.values()])),
            'mean_common_execution_s':float(np.mean([rows[n]['execution_s'] for n in common])) if common else None,
            'mean_common_wall_s':float(np.mean([rows[n]['wall_s'] for n in common])) if common else None,
            'sum_common_decisions':sum(rows[n]['decisions'] for n in common),
            'sum_common_expanded':sum(rows[n]['expanded'] for n in common)}
    compact = {}
    for label,rows in reports.items():
        compact[label] = [{k:r.get(k) for k in ('task_id','is_success','execution_s','wall_s',
            'decisions','expanded','finalizations','first_solution_s','selection_s','observation_s',
            'finalization_s','reason','stop_reason','candidate_generation')} |
            {'audit':r['finalization']['audit'] if r['is_success'] else None,
             'mode_switch_count':r['finalization']['timing']['mode_switch_count'] if r['is_success'] else None}
            for r in rows.values()]
    metrics = {'scope':'offline_engineering_development_not_blind_test_or_ros',
        'summaries':summaries,'common_success_tasks':common,'config':configs[0],
        'evaluations':files,'model_manifests':manifests,'inputs':inputs,'tasks':compact,
        'figure_semantics':'certified timed samples and physical body footprints; not ROS Marker screenshots'}
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    os.environ.setdefault('MPLCONFIGDIR',str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Polygon
    colors = ['#00a8e8','#ee9200','#b83cff']

    def path(ax,name,label):
        problem,row = problems[name],reports[label][name]
        draw_problem(ax,problem)
        ax.tick_params(labelsize=7)
        if row['is_success']:
            trace = np.asarray(row['finalization']['timed_trace'])
            xy = trace[:,1:3]
            ax.add_collection(LineCollection(np.stack([xy[:-1],xy[1:]],axis=1),
                colors=[colors[int(m)] for m in trace[:-1,5]],linewidths=2))
            last = None
            for r in trace:
                _,x,y,yaw,_,mode = r
                if last is not None and math.hypot(x-last[0],y-last[1])<.5*problem.length and \
                    abs(math.atan2(math.sin(yaw-last[2]),math.cos(yaw-last[2])))<math.radians(20):
                    continue
                last = (x,y,yaw)
                c,s = math.cos(yaw),math.sin(yaw)
                box = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[problem.length/2,problem.width/2]
                box = box@np.array([[c,s],[-s,c]])+np.array([x,y])
                ax.add_patch(Polygon(box,fill=False,edgecolor=colors[int(mode)],lw=.65,alpha=.7))
            status = f"PASS T={row['execution_s']:.2f}s / {row['decisions']} decisions"
        else:
            status = 'FAIL '+row['reason']
        ax.set_title(f'{name.split("-")[0]} / {label}\n{status}',fontsize=9)

    chosen = labels[-1]
    fig,axes = plt.subplots(4,3,figsize=(14,17),constrained_layout=True)
    for ax,name in zip(axes.flat,problems):path(ax,name,chosen)
    for ax in list(axes.flat)[len(problems):]:ax.axis('off')
    fig.suptitle(f'{chosen}: offline planning / solid paths passed full audit\n'
                 'A cyan; O orange; Spin purple; green start; red goal; rectangles are vehicle footprints')
    fig.savefig(output/'certified_paths.png',dpi=170);plt.close(fig)
    wanted = [n for n in problems if n[:5] in ('map02','map04','map05','map09','map16')]
    fig,axes = plt.subplots(len(wanted),len(labels),figsize=(6*len(labels),4*len(wanted)),
                          squeeze=False,constrained_layout=True)
    for i,name in enumerate(wanted):
        for j,label in enumerate(labels):path(axes[i,j],name,label)
    fig.suptitle('Same inputs and budgets / offline certified timed trajectories')
    fig.savefig(output/'long_route_comparison.png',dpi=150);plt.close(fig)
    fig,axes = plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
    for ax,key,title in zip(axes,('successes','sum_common_decisions','mean_common_execution_s'),
                           ('Engineering successes / 11','Decisions on common successes','Execution time on common successes [s]')):
        values = [summaries[l][key] for l in labels]
        ax.bar(labels,values,color=['#718096','#057a9a','#d27c00'][:len(labels)])
        ax.set_title(title,fontsize=10);ax.grid(axis='y',alpha=.2)
        for i,v in enumerate(values):ax.text(i,v,f'{v:.2f}' if key.startswith('mean') else str(v),ha='center',va='bottom')
        ax.set_ylim(0,max(values)*1.2)
    fig.suptitle('Development maps / common successful tasks: '+', '.join(n[:5] for n in common))
    fig.savefig(output/'comparison.png',dpi=180);plt.close(fig)
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
