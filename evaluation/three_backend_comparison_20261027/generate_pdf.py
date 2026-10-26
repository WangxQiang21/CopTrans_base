from pathlib import Path
import json
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph,Table,TableStyle
from reportlab.lib.styles import ParagraphStyle
from PIL import Image

ROOT=Path(__file__).resolve().parent
DEST=ROOT/'output/pdf/三分支算法对比汇报_20261027.pdf';DEST.parent.mkdir(parents=True,exist_ok=True)
M=json.loads((ROOT/'metrics.json').read_text());S=M['summary'];RL=json.loads((ROOT/'rl_progress.json').read_text());W,H=841.89,595.28
pdfmetrics.registerFont(TTFont('CN','/usr/share/fonts/truetype/arphic-gbsn00lp/gbsn00lp.ttf'))
pdfmetrics.registerFont(TTFont('CNH','/usr/share/fonts/truetype/arphic-gbsn00lp/gbsn00lp.ttf'))
c=canvas.Canvas(str(DEST),pagesize=(W,H));c.setTitle('三个规划基线与强化学习分支：进展及实测对比');c.setAuthor('CopTrans / whut_thesis')
NAVY=colors.HexColor('#173a52');TEAL=colors.HexColor('#168c76');MUTED=colors.HexColor('#536579')
def para(text,x,y,width,size=12,leading=None,color=NAVY,font='CN'):
 st=ParagraphStyle('cn',fontName=font,fontSize=size,leading=leading or size*1.45,textColor=color,wordWrap='CJK',spaceAfter=0)
 p=Paragraph(text,st);ww,hh=p.wrap(width,1000);p.drawOn(c,x,y-hh);return y-hh

def begin(n,title,subtitle=''):
 c.setFillColor(TEAL);c.rect(32,H-53,5,24,fill=1,stroke=0)
 para(title,48,H-29,W-85,23,27,font='CNH')
 if subtitle:para(subtitle,48,H-69,W-90,10.5,color=MUTED)
 c.setStrokeColor(colors.HexColor('#dce6ec'));c.line(32,34,W-32,34)
 para('CopTrans · 算法基线与强化学习进展 · 2026-10-27',34,25,650,8.5,color=MUTED)
 c.setFont('Helvetica',9);c.setFillColor(MUTED);c.drawRightString(W-34,17,f'{n} / 9')

def img(name,x,y,width,height):
 path=ROOT/name;im=Image.open(path);scale=min(width/im.width,height/im.height);iw,ih=im.width*scale,im.height*scale
 c.drawImage(str(path),x+(width-iw)/2,y-ih,iw,ih,mask='auto');return y-ih

def table(data,x,y,width,widths=None,font=10,row_h=23):
 t=Table(data,colWidths=widths or [width/len(data[0])]*len(data[0]),rowHeights=[row_h]*len(data))
 t.setStyle(TableStyle([('FONTNAME',(0,0),(-1,-1),'CN'),('FONTSIZE',(0,0),(-1,-1),font),('TEXTCOLOR',(0,0),(-1,0),colors.white),('BACKGROUND',(0,0),(-1,0),NAVY),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.HexColor('#edf4f6'),colors.white]),('ALIGN',(0,0),(-1,-1),'CENTER'),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LINEBELOW',(0,0),(-1,0),.6,NAVY),('BOTTOMPADDING',(0,0),(-1,-1),4)]));tw,th=t.wrap(width,1000);t.drawOn(c,x,y-th);return y-th

def card(x,y,w,value,label):
 c.setFillColor(colors.HexColor('#edf6f4'));c.roundRect(x,y-79,w,79,8,fill=1,stroke=0)
 para(value,x+15,y-10,w-30,23 if len(value)>12 else 27,31,color=TEAL,font='CNH');para(label,x+15,y-49,w-30,11,color=MUTED)

begin(1,'三个规划基线与强化学习分支开发进展','汇报版 | 三基线：33 次实际 ROS 运行 | 第四版：规则与学习的独立离线前端')
for x,w,v,label in [(35,241,'3 + 1','三个 ROS 基线 + RL 离线原型'),(300,241,'33 次运行','独立 ROS 任务，均有输入与日志'),(565,241,'9 / 11','V2 精确末端与安全审核通过')]:card(x,H-102,w,v,label)
img('branch_progress.png',34,H-195,W-68,245)
para('已有改进分别减少了搜索工作量、改善了曲率平滑效果、降低了重定时重复计算。上述收益限定于各自配对用例和阶段；本轮整图对比结果见后页。',42,115,W-84,12,18)
c.showPage()

begin(2,'三种方法的特点','共用车辆、地图与 ROS 基础设施；每个分支围绕各自论文思路发展')
info=[('Legacy · multimodal-hybrid-astar','模式感知 Hybrid A* 搜索','将 A / O / Spin 模式、方向和运动历史放入搜索；在搜索提出的结构上继续优化。新加入保守代价下界剪枝，减少不能改善当前解的分支。','参考论文06：Hybrid A* Path Planning with Multi-Modal Motion Extension for Four-Wheel Steering Mobile Robots'),('V1 · mode-path-constant-primitive','模式序列决策 + 分段曲率平滑','先由恒定原语提出模式/方向序列，再对 Ackermann 模式段做曲率二次平滑、边界与末端投影。保留已认证原候选，新候选审核后参与竞争。','参考论文08：A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making'),('V2 · continuous-mode-path','连续控制轨迹 + 段级时间求解复用','用连续曲率/斜移角生成可执行轨迹，按模式事件重配置。相同模式段复用时间求解，每条候选仍独立验碰；当前是继续优化与接入 RL 的平台候选。','参考论文12：From Multi-Modal Paths to Executable Trajectories: A Trajectory Planning Framework for 4WIS Robots')]
y=H-110
for name,concept,body,ref in info:
 c.setFillColor(colors.HexColor('#f0f5f8'));c.roundRect(34,y-126,W-68,126,8,fill=1,stroke=0)
 para(name,47,y-9,W-94,14,20,color=TEAL,font='CNH');para(concept,47,y-34,W-94,13,18,font='CNH');para(body,47,y-60,W-94,11,16);para(ref,47,y-103,W-94,8,11,color=MUTED);y-=140
para('均为项目实现对论文架构的借鉴与扩展；本轮关闭 joint / CBC，不宣称完整复现或全局最优。',42,53,W-84,10,color=MUTED)
c.showPage()

begin(3,'当前版本：V2 在 9 张图上满足精确末端','同图地图栅格、比例、起终点与车辆一致；本轮三个分支实际都启用了双向候选竞争')
img('success_by_map.png',32,H-105,W-64,282)
data=[['指标','Legacy','V1','V2'],['原生合同成功','9 / 11','6 / 11','9 / 11'],['再加 2 cm / 1° 末端要求','1 / 11','6 / 11','9 / 11'],['成功案例规划耗时中位数',f"{S['legacy']['median_planning_s']:.2f} s",f"{S['mode_path_v1']['median_planning_s']:.2f} s",f"{S['mode_path_v2']['median_planning_s']:.2f} s"]]
table(data,48,198,W-96,widths=[310,110,110,W-96-530],font=11,row_h=25)
para('原生用例末端容差为 0.16 m / 0.14 rad；V1/V2 内部还执行精确末端与共同运动学动态审核。Legacy 的原生时间合同较弱；额外末端通过数不代表共同动态认证。每图仅一次运行。',45,85,W-90,10,14,color=MUTED)
c.showPage()

begin(4,'map04：实际 ROS 轨迹与安全走廊显示','矩形直接取自 /plan/corridor_markers；轨迹取自 /plan/timed_trajectory；保持原分支发布密度')
img('map04_trajectory_overlay.png',18,H-105,W-36,349)
para('V2 相对 V1：执行时间 46.83 → 40.75 s，缩短约 13.0%；模式切换（含起点重配置）11 → 9 次。两者精确末端和整车扫掠通过。该例 V1 规划耗时略低于 V2（10.87 / 11.55 s）。',42,123,W-84,13,19)
para('图中车体矩形与安全裕度为实际发布显示；它们不自动等同优化硬约束。所有成功路线均有独立 5 mm 整车扫掠记录。Legacy 使用另一原生时间合同，时长不作跨合同排名。',42,67,W-84,9.5,13,color=MUTED)
c.showPage()

begin(5,'同任务配对：V2 平均执行时间缩短 9.3%','V1 / V2 使用同一动态 profile；共同成功任务为 map01、02、04、05、09、16')
img('successful_case_metrics.png',30,H-108,W-60,300)
card(40,181,235,'21.65 → 19.63','平均执行时间 / s（6 个共同任务）')
card(303,181,235,'5 / 6 图更短','另在 map03 / 06 / 15 增加成功结果')
card(566,181,235,'9.3%','共同任务平均执行时间下降')
para('map09 是例外：V1 29.84 s，V2 30.00 s。这里报告配对案例平均值，未宣称 V2 每张图都更快；墙钟耗时为单次顺序运行观测。',42,77,W-84,10.5,15,color=MUTED)
c.showPage()

begin(6,'map16：地库预设任务的真实运行结果','三分支均完成原生规划；图像由本次 ROS 发布数据渲染，与 RViz 使用同一组矩形和轨迹坐标')
img('maps/map16_corridors.png',18,H-105,W-36,349)
para('V1 / V2 的共同合同执行时长为 16.61 / 15.69 s，V2 缩短约 5.5%。V2 同时通过精确末端、整车扫掠、方向和运动学动态审核。',42,123,W-84,13,19)
para('11 张地图均已生成各自的三分支走廊对比图；完整图片与逐例日志由文字报告链接。失败任务只显示预设起终点，不绘制不存在的成功轨迹。',42,67,W-84,10,14,color=MUTED)
c.showPage()

begin(7,'完整结果与下一步','本轮当前版本数据；完整方法说明、输入身份与图片链接见 REPORT.md')
branches=[S[b]['branch'] for b in ['legacy','mode_path_v1','mode_path_v2']];maps=sorted(set(r['map_id'] for r in M['rows']));lookup={(r['map_id'],r['branch']):r for r in M['rows']}
data=[['地图','Legacy 原生 T / s*','V1 T / s','V2 T / s']]
for m in maps:
 values=[]
 for branch in branches:
  r=lookup[m,branch];values.append(f"{r['route_duration_s']:.2f}" if r['status']=='pass' else '搜索预算耗尽' if r['failure_reason']=='search_budget_exhausted' else '优化未认证')
 data.append([m]+values)
table(data,42,H-108,W-84,widths=[95,225,220,W-84-540],font=10.5,row_h=22)
y=H-108-22*12-15
para('后续工作：以 V2 为平台继续改进困难地图搜索，并补重复运行；保留 Legacy / V1 作为方法对照。RL 前端已复用共同基础框架，下一步验证长距离选择并接入 ROS。',42,y,W-84,12,18,font='CNH')
para('说明：三个分支的 map07、map08 都在本轮预算内未成功，不构成无解证明。* Legacy 原生计时较弱，执行时长不与 V1/V2 排名；它的 9 条原生成功路线中，仅 1 条同时满足额外精确末端要求。完整原始 ROS 数据保存在仓库外归档目录。',42,y-60,W-84,9.5,14,color=MUTED)
para('复核：24 条成功轨迹运行时通过 5 mm 整车扫掠；出图检查 1,266,561 个发布矩形边线采样点，障碍命中为 0。638 项源码哈希、11 图输入一致性与 33 次清理记录均已核对。',42,75,W-84,9,12,color=MUTED)
c.showPage()

begin(8,'第四版：规则候选与学习选择的离线前端','rl-mode-path @ 667cd46 | 复用局部运动连接和共同完成器；从地图与起终点开始选择模式和路径分支')
card(35,H-103,241,'38 项通过','学习模块检查，已有保存日志')
card(300,H-103,241,'727 条记录','564 条拟合 / 163 条 map06 留图')
card(565,H-103,241,'100 轮','RTX 4060 CUDA 监督预训练')
para('当前结构：规则提出可行 A / O / Spin 局部连接，学习策略选择模式与路径分支；末端候选交给共同连续化、重定时和独立审核，再按可执行时间比较。',42,H-206,W-84,13,19)
table([['模块','已实现的内容','完成的作用'],['候选生成','A / O / Spin 与 C++ 局部 A 连接','复用运动学、方向与整车检查'],['学习选择','候选共享评分器、残差 PPO 与分支保留','同时决定模式与局部路径连接'],['公共完成器','连续化、计时、几何/动态/方向审核','复用 V2 的基础框架与共同合同'],['训练与数据','GPU 训练、模型保存/重载、参考选择记录','形成可复查的离线训练闭环']],42,315,W-84,widths=[115,350,W-84-465],font=10.5,row_h=29)
para('已从架构方案推进到可运行的独立离线原型。普通学习推理不调用旧 C++ 完整规划器预先求一条路径；规则候选与学习选择可在同一候选、预算和审核框架下比较。',42,145,W-84,12,18,font='CNH')
para('本页读取已保存进度，本次没有恢复训练。38 项为学习模块检查；RL 尚未接入独立 ROS 前端，run_demo.sh 当前仍运行 V2。',42,70,W-84,10,14,color=MUTED)
c.showPage()

begin(9,'强化学习：离线验证与 GPU 训练进展','已保存实验：2026-10-26 至 27 | 左右是不同阶段模型；本页结果未纳入前三版的 33 次 ROS 排名')
img('rl_progress.png',30,H-103,W-60,276)
para('诊断带来的进展',42,202,355,14,20,color=TEAL,font='CNH')
para('离线参考引导在 8 张输入匹配图中通过 6 张，证明 map02 / 06 / 16 存在可行候选链；这有助于把后续改进集中到分支选择。该 6/8 是知道参考路径的诊断，不是 RL 成功率。',42,177,355,11,16)
para('下一阶段',444,202,355,14,20,color=TEAL,font='CNH')
para('先对新 best.pt / latest.pt 做无参考路径的整图评估，再增加长距离决策训练；验证其优于共同规则基线后，接入独立 ROS 推理入口。map04 的连续化/扫掠衔接另行处理。',444,177,355,11,16)
para('当前可汇报结论：离线前端与 GPU 训练链路已完成运行验证，正在进入长距离决策效果验证。既有 PPO 小试尚无学习收益，新监督权重尚未整图评估。',42,82,W-84,11.5,17,font='CNH')
c.showPage();c.save();print(DEST)
