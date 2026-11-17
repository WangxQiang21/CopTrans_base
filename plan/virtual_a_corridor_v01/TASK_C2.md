# C2：Portal中线弱引导与连续A约束优化

## 独立执行上下文与边界

源码基线`23fd16227e75a68f03f3ac7483555063c8189313`，原分支feature/virtual-rigid-formation-a；只在Codex准备并明确交付的工作区执行。记录HEAD/status/前置ACCEPTED，不覆盖现有数据，不自行建分支、提交或推送。P=`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`。新增接口均为待实现建议，不能假定已存在。

三车偏移(1,0)/(0,1)/(0,-1)m从已有合同取，全车正向统一A，p_i=pF+Rψ r_i，车头独立；保留FREE/STRAIGHT、reset、公共时间律、积分证书、三车运动学/动态、连续OBB和plan/context/model绑定；失败不得误播旧成功。旧FR06和历史证据保护。

**本阶段禁止过度防御性编程**：复用既有模块/类型，输入边界必要验证一次，内部使用已验证合同。不增无意义wrapper/validator/adapter/manager/factory/多层抽象，不加无证据fallback、宽泛吞异常、无限重试、重复日志测试或无关重构。每项新增检查/抽象说明对应哪个可复现错误，否则不加；必要数学、安全、身份检查不能因此删除。

Claude实施时仓库根测试环境：
```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:${PYTHONPATH:-}"
```
若ROS overlay未准备，先纯Python，真实ROS标NOT_RUN。只跑定向测试，禁止全量discover覆盖历史输出。

## 前置与文件

C0/C1 ACCEPTED，分C2-A（中线/分配）、C2-B（NLP/连续包含）两次验收。读P/optimizer.py make_initial_guesses/_guess_route_guidance/_numeric_objective/_build_casadi_nlp，pipeline.py plan_virtual_a，contracts.py PlannerConfig/planning_context/VirtualAResult，geometry.py VirtualAPath及C1实际corridor接口。

允许P/corridor.py、optimizer.py、pipeline.py、contracts.py；ros_adapter.py仅必要settings传递；允许`catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`的main参数构造、`catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`新增显式`virtual_a_guidance_mode:=legacy_route|corridor`（拟新增，默认legacy_route），不能改旧backend默认/状态机。新增tests/test_virtual_a_corridor_optimizer.py。trajectory.py仅发现context具体兼容冲突后报告最小方案，不能改顶层schema。禁止改审核物理门槛、时间律、旧模型、legacy_route默认。

## C2-A（完成先停）

M区域，x0=start/xM=goal，xj∈Pj=Cj−1∩Cj。Chebyshev中心初始化；冻结ℓj=max(seed_length,.1m)，L0=max(端点距离,1m)。QP目标Σ||xj+1−xj||²/L0²+.1Σ||(xj+1−xj)/ℓj−(xj−xj−1)/ℓj−1||²+.01Σ||xj−cj||²/L0²；用现有SciPy SLSQP，primal≤1e-8m。失败保留已验Chebyshev折线并记quality，不加QP框架。

拟`make_corridor_guide(chain,start,goal)->GuideRoute`。首版安全折线用于NLP初值；可选局部二次Bezier圆角仅当三个控制点同属所分配凸C时用凸包性质证明安全，否则保留PL，不能声称光滑。去重复阈值1e-8m；负点积只告警，删V点必须重新LOS和区域包含，真实U不得删除。

拟`allocate_corridors(chain,guide,intervals=8)->tuple[(σa,σb,corridor_index)]`。中线各段长累计τ；σ=s/S固定，用τ和j/8并集切区间，一段Hermite内允许多个C，切换点双重约束。去零长段要同步索引；不能跳过C。测试M=1/3/12对N=8全覆盖无倒序/缺口、不等距QP、U保留、不安全圆角拒绝。C2-A过后等待Codex再接NLP。

## C2-B

拟PlannerConfig加guidance_mode=legacy_route|corridor和必要权重/soft-hard设置，默认legacy_route；所有走廊参数在PlannerConfig单一来源，固定常数也在context配置摘要中可重现；默认as_dict保持原编码，非默认进context。扩`solve_open_path_guided(...,corridor_chain=None,corridor_allocation=None)`关键字，旧接口不变；两solver约束和残差诊断同时扩，不能只IPOPT有效。

z=[S,κ0…κ8,η0…η8]、h=S/8；c0=κj,c1=hηj,c2=3Δκ−h(2ηj+ηj+1),c3=−2Δκ+h(ηj+ηj+1)；内部(2c2l+6c3l)/h²=2c2r/h²。D=(1−bκ)²+(aκ)²，实际C_i=(κD+aη)/D^1.5及所有原硬约束不变。

每个分配区间9点Gauss+两端，世界位置=p_start+Rψ0·局部积分，A p≤b−ε。ε默认1e-5m且≤最小portal半径/4，单位法向。端点型初值无旧route0强贴线；新组guide仅初值/弱偏好。J= S/L0+.01L0∫κ²+.02L0³∫η²+.002L0⁵∫u²+.1mean(||p−guide||²/L0²)+Jclear=mean(max(0,target+support−clearance)^2/L0²)，wc首试1、target/field/support复用现有代理；试验权重入配置，旧组维持5e5/100，不混算收益。

拟`audit_path_in_corridors(path,chain,allocation)->dict`，真模型在s区间中点求值，单位n有n·p(s)≤n·p(mid)+Δs/2+ε_position（||p'_F||=1），计浮点guard后≤b才确认，否则二分。先切Hermite/τ边界；深20/总20000/宽1e-5m触限UNCONFIRMED，不以配点安全代替整段。可用更紧方向导数界但必须证明。soft模式只诊断违约，不声称走廊证书。hard未确认则不可发布该模式成功。

pipeline构造链/guide/分配，真实数据放result.search；几何后、时间前做连续包含，再原geometry/dynamics/OBB终审。中心安全但车体碰撞必须拒绝。固定分配不适用报FIXED_ALLOCATION_LIMIT，不能自动增N/变段长/松审核。

## 测试与量化验收

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```
首项拟新增。精确/近直线、盒绕障、障碍双移线镜像、窄L/U各单次诊断，不承诺全成功；硬门：legacy_route现有六冻结场景无新增回归（新corridor逐项如实报告，不强求六场景全成功），至少一条非平凡新走廊解连续包含+三层通过，无效反例0误发布；目标≤.005m/.2°、刚性≤1e-6m，物理/数值限制按C0批准原合同。numpy/CasADi半空间残差≤1e-8m；配点间越界、中心安全而OBB撞障碍负例均拒绝。

交付A/B同配置、guide来源、mapping、包含与三层审核、长度/κ_i峰值；旧模型JSON往返/摘要不变。Codex核一个σ子区间包含证明、世界坐标变换、两solver一致性及context绑定。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
