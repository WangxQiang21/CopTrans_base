# Claude G4：A-only已知可行窄道、地图引导和三车保守碰撞终审

## 0. 前置、上下文和非目标

仅用户明确下达G4、G0–G3有匹配源码的Codex ACCEPTED后执行；只在 `feature/virtual-rigid-formation-a` 隔离worktree。正式祖先 `567bdecba9676313da3bc9a97b82f57b5b569a2b`。不提交、不改旧FR06、不做G5。

固定offset `1:(1,0),2:(0,1),3:(0,-1)m`、车体.738×.500m；统一A正向，F沿psi。初始静止/平行/直行A，k0=eta0=0。FREE终端不强制回直；C2 Hermite；G3已交付统一五次时间与独立动态审核。

本阶段要证明“在地图上自主找到通过三车几何、动态和保守碰撞审核的完整计划”。不需要ROS，不做新混合模式搜索、动态障碍、在线重规划或最优时间。

## 1. 文件和既有接口

新增：

- `P/formation_continuous_a/collision.py`：新权威曲线到旧扫掠接口适配、地图摘要、数值包络。
- `P/formation_continuous_a/pipeline.py`：有限地图路线引导、solve→audit→retime→audit→collision→result。
- `tools/virtual_a_scenes.py`：新场景构造、参考验证和冻结，不能写旧map/FR06目录。
- `tests/test_virtual_a_collision.py`、`tests/test_virtual_a_pipeline.py`、新`tests/fixtures/virtual_a/maps/**`。

可改新`optimizer.py`增加keyword-only地图引导/约束接口，`audit.py`增加碰撞报告绑定，`contracts.py`完整result，`trajectory.py`增加地图摘要字段，benchmark增加G4。保留G2旧调用语义；新增非兼容字段须更新新模型loader测试，不改旧model版本含义。

`P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

原样调用：`FormationCollisionChecker(grid,model,vehicle_shapes,margin_m)`，`static_check(qf,body_headings)`，`swept_check_analytic(segments,policy,trace=None)`；`AnalyticSweepSegment`、`FormationState`、`SweepPolicy`均在旧`formation_collision.py`。旧OBB/SAT/map边界处理保留。

严禁修改旧碰撞器、车辆尺寸/offset/限制、安全裕量、旧map、FR06参考/实验runner或旧测试预期。若发现真实旧碰撞缺陷，暂停相关安全结论并报告，不能暗中patch旧基线。

## 2. 新接口和完整结果

```python
GuideRoute(points: tuple[tuple[float,float],...], source: str, grid_sha256: str)
VirtualAMapRequest(request: VirtualARequest, grid: Grid,
                   planner_config, timing_config, collision_config)

def make_route_guides(grid, start_qf, goal_qf, config) -> tuple[GuideRoute,...]: ...
def solve_open_path(request, config, *, route_guides=(),
                    obstacle_context=None) -> GeometryResult: ...
def make_sweep_segments(plan: VirtualATimedPlan,
                        config) -> tuple[AnalyticSweepSegment,...]: ...
def audit_collision(request, plan, grid, config) -> AuditReport: ...
def plan_virtual_a(map_request: VirtualAMapRequest) -> VirtualAResult: ...
```

map_request是扩展包装，不把地图塞进三个单车规划器。GuideRoute不能带参考曲率/已知可行解/车体独立路径。

完整结果仅在匹配原request、path/model、limits和grid的GEOMETRY/DYNAMICS/COLLISION均passed时`executable=True`。可执行状态`SUCCESS/NO_MOTION`；失败plan/projection清空或明确不可发布，不能保留上一次计划作为本次结果。报告格式复用旧`FormationEvaluation.as_status/metrics/checks`的可读语义，新报告提供相同必要只读访问，不伪造UniformTimedPlan类型。

物理request_sha256保持G2含义；另计算planning_context_sha256，绑定request_sha、实际grid和三类配置。在生成timed model前传`retime_path(...,planning_context=context)`；`audit_geometry(...,planning_context=context)`和`audit_dynamics(...,expected_planning_context=context)`核对同一上下文。碰撞报告/result/model均带该摘要。不得审核后添加grid字段使已审model_sha过期。

## 3. G4-A：先构造并冻结新A-only存在性场景

必须先做本子任务并保存哈希，再运行正式自主算法。参考用于证明该场景有A-only解，不传给plan_virtual_a；修改地图或参考需新scene版本，不能根据自主失败悄悄放宽。

建议三个具体主场景及y镜像：

### S1 渐进曲率90°窄弯

- 参考生产profile：`k(s)=.45*(s/S)²`，`S=3*pi/(2*.45)=10.47197551197 m`，从F=(0,0,0)开始，终点psi=pi/2；用8段Hermite精确表示二次曲率。
- 参考初始k/eta为0、终端FREE，三车统一A；不得插入S。三车中心曲率及轮系先由G2/G3独立审查。
- 地图分辨率.05m，以独立高精度F参考中心线周围距离<=1.60m构造自由走廊（名义宽3.2m）。**仅为建图**将中心线在起点沿负切向、终点沿正切向各延长2m后加圆帽，保证前置car1车体不被终点圆帽裁掉；延长线不加入执行路径，不涉及曲率拼接。地图外边界再留3m占用空间。
- 采用实际栅格+真实三车扫掠确认参考安全；不能仅凭走廊名义宽度断言通过。

### S2 连续A双移线返回窄廊

- 使用生产C2 profile：站点`s=0,1.5,...,12m`，k值`[0,.30,0,-.30,0,-.30,0,.30,0]`，用两端eta=0的夹持三次样条得到全部eta，再装G1 wrapper。
- 这是可序列化的C2场景参考，**不是G1精确sin³**，其端点须独立积分、记录，不照搬G1数值。
- 名义宽2.8m（参考中心线距离<=1.40m）构造自由走廊；.05m栅格、仅建图的起终切向2m延长和3m外边界与S1相同。检查直接F直线候选确实碰撞，否则该地图不足以验证避让，先在冻结前报告并修场景，不进行假“绕行成功”比较。

### S3 房间内交错挡板避让

- 用S2参考及其独立三车OBB包络构造房间边界：x覆盖全部参考车体范围再加2m，y范围至少[-3,3]m。
- 第一挡板取下方障碍：x区间为F(s=4.8)到F(s=6.0)的x范围；上边界设为**该x带内所有参考车体最低y**减0.05m基准裕量再减0.10m余量，下延至房间下墙。
- 第二挡板取上方障碍：x区间为F(s=9.0)到F(s=10.5)的x范围；下边界设为**该x带内所有参考车体最高y**加0.05m再加0.10m，上延至上墙。
- 包络必须覆盖车体角点及区间误差，不只采F中心。栅格化后重新保守审核参考，并确认直接起终直线因第一挡板被拒绝。该构造只建立A-only已知可行基准，不代表通用困难地图。

三类场景各生成镜像：y/theta/k/eta取相应负号、侧车ID几何互换的对应关系，ID本身不改配置。每张地图保存PGM/YAML、origin/resolution、occupancy哈希、固定request和存在性参考。先至少三条主参考/镜像全部通过完整审核，冻结`scene_manifest.json`。

若某建议宽度/挡板栅格化后参考失败，**不能声称场景无解，也不能在自主求解失败后调整地图**。先停在G4-A，向Codex提交具体碰撞位置、名义/栅格差及最小场景修正；由Codex核定冻结后再开始G4-B。不扩大原FR06地图或改旧margin。

## 4. G4-B：有限地图引导和优化内环

直接复用旧Grid、OBB及距离场基础，首版最多3条二维F路线引导。可用8邻接A*、非负距离场代价与对前一路线局部加代价产生少量替代；节点预算/总时间入config。仅规划一个F，不是三个车辆路径。

路线用于初值/拓扑，不能作为执行折线。F点路线通过不保证三车能过；不得用整个编队外接圆硬膨胀作为唯一允许域。全车外接圆可作快速安全充分条件，无法快通时仍检查真实三车。

NLP继续使用G2全部端点、C2、前向和三车曲率硬约束。在配点恢复各车实际theta_i，使用缓存距离场/光滑proxy和必要障碍约束引导；最终一定重新真实矩形扫掠。可按终审碰撞位置增加约束后在原总预算内重解，记录轮次，最多2次，不开发无限修补循环。

默认总软规划预算60s/请求，最多3条路线、每路线使用G2自动种子但受同一总deadline，不得变成3×6×30s的隐式大预算；不承诺严格硬实时返回。分别记录路线生成、NLP、动态审核、碰撞审核及模型构建时间。

不读取S1/S2/S3存在性参考。正式pipeline调用的全部参数须可记录；用patch参考函数一旦被调用即抛错，并让正式规划在没有reference文件的输入目录运行，核查生产import/文件访问。

## 5. 保守扫掠和误差规范

对每个Hermite段或其必要细分`[sa,sb]`，Delta=sb-sa，lambda∈[0,1]：

```text
s=sa+Delta*lambda
segment.station_start=sa; station_end=sb
state_at(lambda)=FormationState(s, path.state_at_s(s).qf,
                               {v.vehicle_id:v.theta for v in path.state_at_s(s).vehicles})
time_at(lambda)=plan.time_at_s(s)
center_speed_bound_m_per_lambda=Delta
formation_rate_bound_rad_per_lambda=Delta*sup(abs(k))
body_rate_bounds_rad_per_lambda[i]=Delta*sup(abs(H_i))
H_i=k+a_i*eta/D_i
```

K/H/D上界由已验收外扩区间算子给出，不能用配点最大值。所有“每米”导数转换成“每lambda”时乘Delta。time_at使用G3真实公共时间逆，不把s冒充秒。

旧checker依据`||p_i_lambda||<=||pF_lambda||+|r_i|*|psi_lambda|`，再加inflated body radius×theta导数生成覆盖；该实现无需重写。

checker基准margin=.05m，SweepPolicy(max_corner_step_m=.01,max_samples_per_segment=200000)。数值误差额外包络至少包括：

`eps_position + (abs(offset)+inflated_body_radius)*eps_psi + inflated_body_radius*eps_relative_heading`。

按各车/区间最大值增加checker margin，不减小.05，不修改车体形状来伪装裕量。边界/角点接触按原checker保守拒绝；预算耗尽不可标无碰撞。

报告保存trace中的s/time/qF/actual_heading、bound及采样数；先static_check起终点，再swept_check_analytic。NO_MOTION只需完整静态地图/车间检查和前阶段合法hold，不能跳过地图。

## 6. 强制测试和定量验收

正例：三类冻结地图及镜像的参考均通过；正式自主pipeline只读map/request，全部六个最小已知可行请求产生三层审核通过结果。要求是阶段目标，不是已验证的性能保证；失败则返工/报告，不删失败数据。

负例：

1. 地图中薄障碍放在两个粗采样时刻之间，粗样本未撞但真实扫掠必须拒绝。
2. 车体角点碰撞但中心不碰撞，必须拒绝；前车theta_i≠psi时须用真实车头。
3. 相同模型换地图/限值/目标或旧审核摘要，必须绑定失败或重新审核。
4. 故意把Delta因子漏掉的适配结果，由独立检查发现上界错误，不允许测试仅trust segment自报bound。
5. 整群外接圆无法快通但真实OBB可通的构造例，最终流程不应仅因大圆拒绝。
6. 扫掠样本预算设过小，返回预算失败且不可执行。
7. 地图边界、unknown占用、起点/目标碰撞、NO_MOTION碰撞、solver假成功、无合格候选和超预算。

验收门槛：刚性<=1e-6m，终点<=.005m/.2°，原动态限制全部通过；碰撞裕量/步长不放宽；连续域导数界和误差包络有证据，不能只图像目测。实际栅格净空或可信下界须注明度量方法，不能把名义W/2减几何包络写成精确栅格净空。

保存全部尝试及分层耗时。不将有限集合通过写成一般成功率/完备性，不将旧FR06混合模式参考当新可行性或性能证据。

## 7. 顺序和命令

G4-A先场景/存在性冻结；G4-B再扫掠适配+真实负例及地图引导；G4-C最后端到端自主pipeline。若A未通过，不做B的正式成功率实验。

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G4/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
set -o pipefail
python3 -B tools/virtual_a_scenes.py --output "$VA_RUN/scenes" --verify-references
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collision.py' -v 2>&1 | tee "$VA_RUN/logs/collision.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v 2>&1 | tee "$VA_RUN/logs/pipeline.log"
python3 -B tools/virtual_a_benchmark.py --stage G4 --scene-manifest "$VA_RUN/scenes/scene_manifest.json" --output "$VA_RUN/benchmark"
```

工具/fixture加入时保证G2/G3命令仍可用；关键受影响回归必须跑。证据目录新建且唯一，旧历史完全只读。

## 8. 交付和Codex独立验收

交付地图/场景冻结资料、独立参考与原请求、正式无reference输入副本、实际自主结果、权威timed model、三层audit与request/grid/limits/model摘要、扫掠trace/bounds、所有负例/失败、求解与碰撞分层计时、保护哈希及`CLAUDE_REPORT.md`。

Codex独立：

- 直接读取PGM重新建Grid、独立核对offset/limits和参考只含A、不含S准备。
- 从原请求重审至少每类一条参考/自主模型，镜像按车ID对应，不信scene“已知可行”标签。
- 检查参考路径未泄漏，独立只给map/端点跑至少一条非平凡请求。
- 独立推导lambda导数上界，检查真实车头、积分误差和基准margin叠加。
- 构造/复核薄障碍、角点和预算负例，确认几何/动态通过不越权变成碰撞通过。
- 核对总预算没有乘法膨胀、失败数据没删除，旧FR06和G1–G3限制未改。

**停止：完成G4报告后等待Codex ACCEPTED和用户下达G5。不启动ROS、不额外做优化平台、不提交Git。**
