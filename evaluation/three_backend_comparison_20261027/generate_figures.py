#!/usr/bin/env python3
"""Render actual ROS timed trajectories and published LINE_LIST corridors."""
import argparse,hashlib,json,math,pathlib,re,shutil,statistics
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap

parser=argparse.ArgumentParser();parser.add_argument('--raw',type=pathlib.Path,required=True);parser.add_argument('--output',type=pathlib.Path,required=True);args=parser.parse_args()
RAW=args.raw.resolve();OUT=args.output.resolve();OUT.mkdir(parents=True,exist_ok=True);(OUT/'maps').mkdir(exist_ok=True)
META=json.loads((RAW/'manifest.json').read_text());assert META.get('complete')
BACKENDS=['legacy','mode_path_v1','mode_path_v2'];LABELS={'legacy':'Legacy · Hybrid A*','mode_path_v1':'V1 · 模式序列与分段平滑','mode_path_v2':'V2 · 连续模式轨迹'}
COLORS={0:'#00a9e9',1:'#f08b14',2:'#c83cef'}
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def number(pattern,text):
 m=re.search(pattern,text or '');return float(m.group(1)) if m else None

def map_data(run_dir,run):
 mdir=run_dir/('runtime_map' if (run_dir/'runtime_map/map.yaml').exists() else 'input_map')
 meta=yaml.safe_load((mdir/'map.yaml').read_text());image=np.asarray(Image.open(mdir/meta['image']).convert('L'))
 # PGM row 0 is the TOP; ROS OccupancyGrid row 0 is the BOTTOM.
 occ=np.flipud(image)<255-float(meta.get('free_thresh',.196))*255
 assert hashlib.sha256(np.asarray(occ,dtype=np.uint8).tobytes()).hexdigest()==run['case_data']['optimizer_map_signature']['sha256']
 res=float(np.float32(meta['resolution']));ox,oy=meta['origin'][:2];H,W=occ.shape
 return occ,res,ox,oy,(ox,ox+W*res,oy,oy+H*res),mdir

def hits(points,occ,res,ox,oy):
 if not len(points):return 0
 pts=np.asarray(points);ix=np.floor((pts[:,0]-ox)/res).astype(int);iy=np.floor((pts[:,1]-oy)/res).astype(int)
 valid=(ix>=0)&(iy>=0)&(ix<occ.shape[1])&(iy<occ.shape[0]);bad=np.ones(len(pts),dtype=bool);bad[valid]=occ[iy[valid],ix[valid]];return int(bad.sum())

def segments(marker):
 points=np.asarray(marker['points'],float);assert len(points)%2==0
 q=marker['pose_orientation_xyzw'];yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));c,s=math.cos(yaw),math.sin(yaw)
 xy=points[:,:2]@np.asarray([[c,s],[-s,c]])+np.asarray(marker['pose_position'][:2])
 return xy.reshape(-1,2,2)

rows={};checks=[]
for backend in BACKENDS:
 rows[backend]={}
 for case in META['cases']:
  directory=RAW/backend/case;run=json.loads((directory/'run.json').read_text());p=run.get('probe_result',{});q=p.get('quality_report',{});case_d=run['case_data'];mapid=pathlib.Path(case_d['map_dir']).name.split('-')[0]
  passed=bool(run.get('status')=='pass' and p.get('timed_sweep_clear') and p.get('terminal_ok') and p.get('independent_direction',{}).get('pass_check') and p.get('independent_dynamics',{}).get('pass_check'))
  strict=bool(passed and p['position_error']<=.02 and abs(p['heading_error'])<=math.pi/180)
  st=number(r'(?<![\w])seconds=([\d.]+)',p.get('search'));ot=number(r'\btime=([\d.]+)s',p.get('optimizer'));trace=np.asarray(p.get('timed_trace',[]),float)
  reason=q.get('reason') or ('search_budget_exhausted' if 'search_budget_exhausted' in (p.get('search') or '') else 'no_certified_candidate')
  if reason=='no_feasible_candidate_in_search_budget':reason='optimizer_no_certified_candidate'
  metrics=q.get('selected',{}).get('metrics',{})
  row={'map_id':mapid,'case':case,'branch':META['branches'][backend]['branch'],'commit':META['branches'][backend]['commit'],'status':'pass' if passed else 'fail','strict_endpoint_pass':strict,'route_duration_s':p.get('duration') if passed else None,'search_seconds':st,'optimizer_seconds':ot,'planning_seconds':st+ot if st is not None and ot is not None else None,'mode_switches':len(p['mode_runs'])-1 if passed else None,'position_error_m':p.get('position_error'),'yaw_error_deg':math.degrees(abs(p['heading_error'])) if 'heading_error' in p else None,'path_length_m':float(np.linalg.norm(np.diff(trace[:,1:3],axis=0),axis=1).sum()) if passed else None,'min_body_clearance_m':metrics.get('min_body_clearance_m'),'reverse_length_m':p.get('independent_direction',{}).get('reverse_distance_m'),'dynamics_contract':p.get('dynamics_contract'),'dynamic_profile':q.get('selected_timing',{}).get('profile'),'failure_reason':None if passed else reason,'swept_clear':p.get('timed_sweep_clear'),'sweep_samples':p.get('sweep_samples'),'cleanup_ok':run.get('cleanup',{}).get('all_started_processes_stopped') and run.get('cleanup',{}).get('port_released'),'raw_run_dir':str(directory),'probe_sha256':sha(directory/'probe_result.json'),'map_scale':case_d['map_scale'],'actual_direction_policy':re.search(r'direction_policy=([^ ]+)',p.get('search') or '').group(1)}
  if passed:
   row['internal_mode_switches']=row['mode_switches']
   row['initial_mode_switches']=int(int(trace[0,5])!=0)
   row['mode_switches']+=row['initial_mode_switches']
   if q.get('selected_timing',{}).get('mode_switch_count') is not None:
    assert row['mode_switches']==q['selected_timing']['mode_switch_count']
  rows[backend][mapid]=(row,run,p)
  occ,res,ox,oy,extent,mdir=map_data(directory,run)
  chk={'backend':backend,'map_id':mapid,'occupancy_signature_matches_run':True,'map_yaml_sha256':sha(mdir/'map.yaml'),'probe_sha256':row['probe_sha256'],'source_topic':'/plan/corridor_markers','marker_count':len(p.get('corridor_display',{}).get('markers_world',[])),'center_occupied_hits':None,'published_marker_edge_occupied_hits':None,'marker_edge_samples':0}
  if passed:
   chk['center_occupied_hits']=hits(trace[:,1:3],occ,res,ox,oy);allpts=[]
   for marker in p['corridor_display']['markers_world']:
    assert marker['frame_id']=='map'
    for a,b in segments(marker):
     n=max(1,math.ceil(float(np.linalg.norm(b-a))/min(.005,res/2)));allpts.extend(a+np.linspace(0,1,n+1)[:,None]*(b-a))
   chk['published_marker_edge_occupied_hits']=hits(np.asarray(allpts),occ,res,ox,oy);chk['marker_edge_samples']=len(allpts)
   assert chk['center_occupied_hits']==0,chk
   assert chk['published_marker_edge_occupied_hits']==0,chk
   assert chk['marker_count']>0
  checks.append(chk)

MAPS=sorted(rows['legacy'])
def draw(ax,backend,mapid,small=False):
 row,run,p=rows[backend][mapid];directory=pathlib.Path(row['raw_run_dir']);occ,res,ox,oy,extent,_=map_data(directory,run)
 ax.imshow(np.where(occ,.07,.82),cmap='gray',vmin=0,vmax=1,origin='lower',extent=extent,interpolation='nearest')
 if row['status']=='pass':
  markers=p['corridor_display']['markers_world']
  for margin in [True,False]:
   for marker in markers:
    ism='margin' in marker['namespace']
    if ism!=margin:continue
    color='#e8e8e8' if ism else COLORS.get(marker['id'],'#36ad70')
    ax.add_collection(LineCollection(segments(marker),colors=color,linewidths=.35 if ism else .85,alpha=.8 if ism else .9,zorder=2))
  trace=np.asarray(p['timed_trace']);segs=np.stack([trace[:-1,1:3],trace[1:,1:3]],axis=1)
  ax.add_collection(LineCollection(segs,colors=[COLORS[int(m)] for m in trace[:-1,5]],linewidths=1.6,zorder=3))
  ax.scatter(trace[-1,1],trace[-1,2],s=9,c='#f92332',zorder=5)
  subtitle=f"T={row['route_duration_s']:.2f}s | 切换 {row['mode_switches']} | 末端误差 {row['position_error_m']*100:.2f}cm"
 else:subtitle='搜索预算耗尽' if row['failure_reason']=='search_budget_exhausted' else '搜索后未得到认证轨迹'
 for pose,color in [(run['case_data']['start'],'#0db54f'),(run['case_data']['goal'],'#ea242b')]:
  length=min(extent[1]-extent[0],extent[3]-extent[2])*.065;x,y,yaw=pose
  ax.arrow(x-length*.4*math.cos(yaw),y-length*.4*math.sin(yaw),length*.8*math.cos(yaw),length*.8*math.sin(yaw),width=length*.10,head_width=length*.30,head_length=length*.25,length_includes_head=True,color=color,zorder=6)
 ax.set_xlim(extent[:2]);ax.set_ylim(extent[2:]);ax.set_aspect('equal');ax.set_xticks([]);ax.set_yticks([])
 ax.set_title((mapid+' | ' if small else LABELS[backend]+'\n')+subtitle,fontsize=8 if small else 10,color='#263449' if row['status']=='pass' else '#a12e27',pad=9)

for mapid in MAPS:
 fig,axs=plt.subplots(1,3,figsize=(16,6))
 for ax,b in zip(axs,BACKENDS):draw(ax,b,mapid)
 fig.suptitle(mapid+' · 三分支实际 ROS 轨迹与走廊显示',fontsize=17,fontweight='bold',y=.98)
 fig.text(.5,.035,'青=A；橙=O；紫=Spin；浅灰=车体安全裕度边界。矩形来自 /plan/corridor_markers，绿箭头=起点，红箭头=目标。',ha='center',fontsize=9)
 fig.subplots_adjust(top=.82,bottom=.11,left=.02,right=.98,wspace=.10);fig.savefig(OUT/'maps'/(mapid+'_corridors.png'),dpi=170,bbox_inches='tight');plt.close(fig)
 if mapid=='map04':shutil.copy2(OUT/'maps'/(mapid+'_corridors.png'),OUT/'map04_trajectory_overlay.png')
for b in BACKENDS:
 fig,axs=plt.subplots(4,3,figsize=(12,15))
 for ax,m in zip(axs.flat,MAPS):draw(ax,b,m,True)
 axs.flat[-1].axis('off');fig.suptitle(LABELS[b]+' · 全部 11 图 ROS 结果',fontsize=17,fontweight='bold',y=.995)
 fig.subplots_adjust(hspace=.28,wspace=.12,top=.95,left=.015,right=.985,bottom=.02);fig.savefig(OUT/(b+'_all_maps.png'),dpi=150,bbox_inches='tight');plt.close(fig)

summary={}
for b in BACKENDS:
 rr=[v[0] for v in rows[b].values()];ps=[r for r in rr if r['status']=='pass'];med=lambda k:statistics.median([r[k] for r in ps if r[k] is not None])
 summary[b]={'branch':META['branches'][b]['branch'],'commit':META['branches'][b]['commit'],'native_pass':len(ps),'strict_endpoint_pass':sum(r['strict_endpoint_pass'] for r in rr),'median_planning_s':med('planning_seconds'),'median_duration_s':med('route_duration_s'),'median_switches':med('mode_switches'),'median_length_m':med('path_length_m'),'median_min_body_clearance_m':med('min_body_clearance_m') if any(r['min_body_clearance_m'] is not None for r in ps) else None,'max_position_error_m':max(r['position_error_m'] for r in ps),'max_yaw_error_deg':max(r['yaw_error_deg'] for r in ps)}
common=[m for m in MAPS if all(rows[b][m][0]['status']=='pass' for b in ['mode_path_v1','mode_path_v2'])]
v1=np.array([rows['mode_path_v1'][m][0]['route_duration_s'] for m in common]);v2=np.array([rows['mode_path_v2'][m][0]['route_duration_s'] for m in common]);ratios=v1/v2
paired={'maps':common,'v1_duration_s':v1.tolist(),'v2_duration_s':v2.tolist(),'median_v1_over_v2':float(np.median(ratios)),'v2_shorter_cases':int((v2<v1).sum()),'mean_v1_s':float(v1.mean()),'mean_v2_s':float(v2.mean()),'relative_mean_reduction':float(1-v2.sum()/v1.sum())}
metrics={'schema':'current_three_branch_ros_comparison_v2','date':'2026-10-27','raw_root':str(RAW),'manifest_sha256':sha(RAW/'manifest.json'),'cases_sha256':sha(RAW/'cases.yaml'),'summary':summary,'paired_v1_v2':paired,'rows':[rows[b][m][0] for m in MAPS for b in BACKENDS],'image_checks':checks,'notes':{'native_terminal':'case tolerance 0.16m / 0.14rad; V1/V2 additionally use internal strict endpoint','strict_endpoint':'0.02m / 1 degree, endpoint only; does not mean common dynamic certification','corridor_source':'Published ROS MarkerArray; footprints and margin display, not optimization constraints','legacy_timing':'legacy_native_speed_only; do not rank durations against V1/V2','sequential':True,'runs_per_case':1,'mode_switch_count':'Includes initial A-to-first-mode setup; internal changes and initial setup also recorded separately.'}}
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n');(OUT/'render_checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n')

fig,(ax,hm)=plt.subplots(1,2,figsize=(14,5),gridspec_kw={'width_ratios':[1,1.8]});x=np.arange(3)
a=ax.bar(x-.18,[summary[b]['native_pass'] for b in BACKENDS],width=.36,color='#568ba8',label='原生合同通过');a2=ax.bar(x+.18,[summary[b]['strict_endpoint_pass'] for b in BACKENDS],width=.36,color='#2aa874',label='另加 2cm / 1° 末端要求')
for bars in [a,a2]:
 for rect in bars:ax.text(rect.get_x()+rect.get_width()/2,rect.get_height()+.12,str(int(rect.get_height())),ha='center',fontweight='bold')
ax.set_xticks(x);ax.set_xticklabels(['Legacy','V1','V2']);ax.set_ylim(0,11.9);ax.set_yticks(range(0,12,2));ax.set_ylabel('地图数 / 11');ax.legend(fontsize=8,loc='upper left');ax.grid(axis='y',alpha=.15)
mat=[[1 if rows[b][m][0]['strict_endpoint_pass'] else .5 if rows[b][m][0]['status']=='pass' else 0 for m in MAPS] for b in BACKENDS]
hm.imshow(mat,origin='upper',aspect='auto',vmin=0,vmax=1,cmap=ListedColormap(['#eab3ad','#f3d17f','#83c99c']));hm.set_xticks(range(11));hm.set_xticklabels(MAPS,rotation=45,ha='right');hm.set_yticks(range(3));hm.set_yticklabels(['Legacy','V1','V2'])
for i in range(3):
 for j in range(11):hm.text(j,i,'✓' if mat[i][j]==1 else '△' if mat[i][j]==.5 else '×',ha='center',va='center',fontsize=13)
hm.set_title('绿：通过且末端精确；黄：只满足宽松末端；红：失败',fontsize=10)
fig.suptitle('2026-10-27 当前三分支重跑：11 图 × 3 后端',fontsize=15,fontweight='bold');fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/'success_by_map.png',dpi=180,bbox_inches='tight');plt.close(fig)
fig,(a,b)=plt.subplots(1,2,figsize=(12,4.6));x=np.arange(len(common));a.bar(x-.18,v1,width=.36,color='#e68a00',label='V1');a.bar(x+.18,v2,width=.36,color='#1484c6',label='V2');a.set_xticks(x);a.set_xticklabels(common);a.set_ylabel('轨迹执行时间 / s');a.legend();a.grid(axis='y',alpha=.15);a.set_title('共同成功的 6 图：相同时序合同')
p1=[rows['mode_path_v1'][m][0]['planning_seconds'] for m in common];p2=[rows['mode_path_v2'][m][0]['planning_seconds'] for m in common];b.bar(x-.18,p1,width=.36,color='#e68a00',label='V1');b.bar(x+.18,p2,width=.36,color='#1484c6',label='V2');b.set_xticks(x);b.set_xticklabels(common);b.set_ylabel('搜索 + 优化耗时 / s');b.legend();b.grid(axis='y',alpha=.15);b.set_title('同 6 图规划耗时（单次顺序运行）')
fig.suptitle('V1 与 V2 的配对比较',fontsize=15,fontweight='bold');fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/'successful_case_metrics.png',dpi=180,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(12,6.3))
for ax,backend in zip(axs,BACKENDS):
 row,run,p=rows[backend]['map04'];tr=np.asarray(p['timed_trace']);ax.plot(tr[:,0],tr[:,4],c='#36556a',lw=1)
 for mode in range(3):
  sel=tr[:,5]==mode;ax.scatter(tr[sel,0],tr[sel,4],s=2.2,color=COLORS[mode],label=['A','O','Spin'][mode])
 ax.set_title(LABELS[backend]+f" | {row['route_duration_s']:.2f}s",fontsize=10,loc='left');ax.set_ylabel('v / (m/s)');ax.grid(alpha=.18);ax.legend(loc='upper right',ncol=3,fontsize=8);ax.set_xlim(0,tr[-1,0])
axs[-1].set_xlabel('轨迹时间 / s');fig.suptitle('map04 · 实际发布的速度与模式',fontsize=15,fontweight='bold');fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'map04_speed.png',dpi=170,bbox_inches='tight');plt.close(fig)
print(json.dumps({'summary':summary,'paired':paired,'image_checks':{'count':len(checks),'pass_center_hits':sum(x['center_occupied_hits'] or 0 for x in checks),'pass_marker_edge_hits':sum(x['published_marker_edge_occupied_hits'] or 0 for x in checks)}},ensure_ascii=False,indent=2))
# Paired improvement evidence from the three branch-specific, previously saved experiments.
import subprocess
progress={}
for backend,paper in [('legacy','paper06'),('mode_path_v1','paper08'),('mode_path_v2','paper12')]:
 commit=META['branches'][backend]['commit'];path='evaluation/'+paper+'/metrics.json'
 blob=subprocess.check_output(['git','-C',str(OUT),'show',commit+':'+path]);d=json.loads(blob)
 if backend=='legacy':
  before=d['before']['expanded_nodes'];after=d['after']['expanded_nodes'];scope='map04：搜索扩展节点';unit='节点';extra={'identical_timed_trace':d['identical_timed_trace']}
 elif backend=='mode_path_v1':
  before=d['before_duration_s'];after=d['after_duration_s'];scope='4 m 曲率振荡定向用例';unit='执行时间 / s';extra={'profile':d['profile']['profile'],'fixture':d['fixture'],'map01_recovered':True}
 else:
  before=d['ros_map04']['before_retime_s']*1000;after=d['ros_map04']['after_retime_s']*1000;scope='map04：重定时阶段耗时';unit='ms';extra={'identical_serialized_trace':d['ros_map04']['identical_serialized_trace'],'microbenchmark_speedup':d['timing_speedup']}
 progress[backend]={'source_commit':commit,'source_path':path,'source_sha256':hashlib.sha256(blob).hexdigest(),'before':before,'after':after,'reduction_percent':(1-after/before)*100,'scope':scope,'unit':unit,'details':extra}
(OUT/'historical_progress.json').write_text(json.dumps({'date':'2026-10-25','note':'These paired improvement experiments have distinct scopes and profiles; they are not the 2026-10-27 cross-branch evaluation.','experiments':progress},ensure_ascii=False,indent=2)+'\n')
fig,axs=plt.subplots(1,3,figsize=(13.3,4.7))
for ax,backend in zip(axs,BACKENDS):
 d=progress[backend];bars=ax.bar([0,1],[d['before'],d['after']],color=['#9baebd','#16a275'],width=.55);ax.set_xticks([0,1]);ax.set_xticklabels(['修改前','修改后']);ax.set_ylabel(d['unit']);ax.set_ylim(0,d['before']*1.26);ax.set_title(LABELS[backend]+'\n'+d['scope'],fontsize=10,pad=9);ax.grid(axis='y',alpha=.15)
 for rect,value in zip(bars,[d['before'],d['after']]):ax.text(rect.get_x()+rect.get_width()/2,value+d['before']*.025,f'{value:,.0f}' if backend=='legacy' else f'{value:.2f}',ha='center',fontsize=10)
 ax.text(.5,.92,f"下降 {d['reduction_percent']:.1f}%",transform=ax.transAxes,ha='center',color='#087c50',fontweight='bold',fontsize=12)
fig.suptitle('三个分支各自的已验证优化收益',fontsize=16,fontweight='bold',y=1.01);fig.text(.5,-.015,'来自各分支 2026-10-25 的配对实验；指标与场景不同，收益限于图中标明的用例/阶段。',ha='center',fontsize=9);fig.tight_layout(rect=(0,.04,1,.94));fig.savefig(OUT/'branch_progress.png',dpi=180,bbox_inches='tight');plt.close(fig)
# Same map16 run, shared crop around the union of all three published routes.
points=[]
for backend in BACKENDS:
 row,run,p=rows[backend]['map16']
 for marker in p['corridor_display']['markers_world']:points.extend(segments(marker).reshape(-1,2))
points=np.asarray(points);lo=points.min(axis=0)-.35;hi=points.max(axis=0)+.35
fig,axs=plt.subplots(1,3,figsize=(16,4.5))
for ax,backend in zip(axs,BACKENDS):
 draw(ax,backend,'map16');ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_title(LABELS[backend]+f"\nT={rows[backend]['map16'][0]['route_duration_s']:.2f}s",fontsize=10,pad=8)
fig.suptitle('map16 · 实际发布走廊的路线区域放大（同一裁剪范围）',fontsize=15,fontweight='bold',y=.99);fig.subplots_adjust(wspace=.10,top=.76,bottom=.08,left=.025,right=.975);fig.savefig(OUT/'map16_corridors_zoom.png',dpi=180,bbox_inches='tight');plt.close(fig)
