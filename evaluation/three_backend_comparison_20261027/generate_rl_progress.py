#!/usr/bin/env python3
"""Summarize the saved RL checkpoint; does not train or evaluate policies."""
from pathlib import Path
import hashlib,json,subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT=Path(__file__).resolve().parent
COMMIT='667cd4661271e7aeee94a154212f341d829b617f'
def git_file(path):return subprocess.check_output(['git','-C',str(OUT),'show',COMMIT+':'+path])
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
checkpoint_path='evaluation/rl/rule_frontend/checkpoint_20261027.json'
metrics_path='evaluation/rl/rule_frontend/metrics.json'
checkpoint_blob=git_file(checkpoint_path);metrics_blob=git_file(metrics_path)
cp=json.loads(checkpoint_blob);m=json.loads(metrics_blob)
train=Path(cp['pretraining']['path']);history_path=train/'history.json';history=json.loads(history_path.read_text())
assert len(history)==100
for name,value in cp['pretraining']['models_sha256'].items():assert digest(train/name)==value
assert digest(train/'manifest.json')==cp['pretraining']['manifest_sha256']
for key in ['rule_val','trained_val','rule_maps_10s','trained_maps_10s']:
 record=m['evaluations'][key];assert digest(Path(record['path'])/'evaluation.json')==record['evaluation_sha256']
summary={
 'schema':'saved_rl_progress_v1','source_branch':'rl-mode-path','source_commit':COMMIT,'saved_date':'2026-10-27','report_action':'read_saved_results_only; training_remains_paused',
 'sources':{checkpoint_path:hashlib.sha256(checkpoint_blob).hexdigest(),metrics_path:hashlib.sha256(metrics_blob).hexdigest(),'training_history_path':str(history_path),'training_history_sha256':digest(history_path)},
 'approach':'Rule-feasible A/O/Spin local candidates; residual policy selects mode and path branch; shared continuous finalizer, retiming and independent audits.',
 'checks_passed':cp['checks']['passed'],'current_online_launcher':cp['current_online_launcher'],
 'planning_evaluations':{key:m['evaluations'][key]['summary'] for key in ['rule_val','trained_val','rule_maps_10s','trained_maps_10s']},
 'oracle_diagnostic':cp['oracle_diagnostic'],'pretraining':cp['pretraining'],'training_history':history,
 'limits':['20/20 short-task feasibility is shared by rules and the PPO pilot, not proof of RL benefit.','Engineering rule and PPO pilot both 3/11; not included in the current 33-run ROS ranking.','Reference-guided 6/8 is an offline oracle diagnosis, not an RL success rate.','Teacher-forced action accuracy is not route completion rate.','The new best/latest supervised weights have not had full-map planner evaluation.','New RL frontend is offline; run_demo.sh still runs V2.']}
(OUT/'rl_progress.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
fig,(a,b)=plt.subplots(1,2,figsize=(12.5,4.8))
x=np.arange(2);rules=[m['evaluations'][k]['summary']['success_rate']*100 for k in ['rule_val','rule_maps_10s']];learned=[m['evaluations'][k]['summary']['success_rate']*100 for k in ['trained_val','trained_maps_10s']]
a.bar(x-.18,rules,width=.36,color='#7f9da7',label='规则候选基线');a.bar(x+.18,learned,width=.36,color='#1688bb',label='512 步 PPO 小试')
a.set_xticks(x);a.set_xticklabels(['固定短程验证','工程地图']);a.set_ylim(0,119);a.set_ylabel('最终审核通过率 / %');a.legend(fontsize=8,loc='upper right');a.set_title('独立离线前端已可运行')
for i in range(2):
 for dx,value in [(-.18,rules[i]),(.18,learned[i])]:a.text(i+dx,value+2,'20/20' if i==0 else '3/11',ha='center',fontsize=10)
a.grid(axis='y',alpha=.18)
epoch=[x['epoch'] for x in history];b.plot(epoch,[x['train_accuracy']*100 for x in history],color='#15956c',label='拟合任务（564 条）');b.plot(epoch,[x['holdout_accuracy']*100 for x in history],color='#e38b29',label='map06 留图（163 条）')
b.set_ylim(0,55);b.set_xlabel('监督预训练轮数');b.set_ylabel('教师动作预测准确率 / %');b.set_title('RTX 4060：完成 100 轮监督预训练');b.grid(alpha=.18);b.legend(fontsize=8)
fig.suptitle('第四版 RL：已保存的离线验证与 GPU 训练进展',fontsize=14,fontweight='bold');fig.text(.5,.03,'左：2026-10-26 规则/PPO 小试；右：2026-10-27 新监督预训练，对应不同阶段模型。',ha='center',fontsize=8.5)
fig.text(.5,.0,'动作预测准确率不是路径到达率；最新监督权重尚未整图评估，尚未证明学习收益。',ha='center',fontsize=8.5)
fig.tight_layout(rect=(0,.09,1,.93));fig.savefig(OUT/'rl_progress.png',dpi=180,bbox_inches='tight');plt.close(fig)
print('RL checkpoint read and verified:',COMMIT,'| 100 epochs | 727 decisions | no training started')
