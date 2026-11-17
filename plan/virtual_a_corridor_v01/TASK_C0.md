# C0：数值修复和修复后旧引导基线

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

## 目标、前置与允许范围

前置：本轮HEAD、Codex准备工作区。分C0-A（配点/失败摘要）与C0-B（同候选审计预算分析）两门，分别完成即停等验收。不加走廊，不改5e5/100路线权重、段数或初值数量。

读P/optimizer.py `_collocation_states/_numeric_objective/_build_casadi_nlp/_numeric_constraint_vector/solve_open_path_guided`；P/audit.py `_integration_plan`；P/integration_bounds.py、bounds.py；`docs/virtual_formation_a/diagnostics_20261116/`。

允许optimizer.py、定向test_virtual_a_optimizer.py，拟新增tests/test_virtual_a_collocation.py及tests/fixtures/virtual_a_corridor/小回归输入；C0-B仅有独立推导支持时局部改audit/integration_bounds/bounds。禁止改原5e-8测试预期、车辆限制、地图、ROS、时间律。

## 步骤和接口

1. 冻结3组请求/地图SHA/完整配置/依赖：局部精确直线(0,0,0)→(4,0,0)；历史近直线(12.57,10.32,-.0297339)→(16.5,10.32,-.0297339)；历史回头弯同起点→(9.694,2.854,3.132)。近直线有约.117m局部横移，不能要求零κ。后两项必须找到历史真实地图，与uturn summary的grid SHA核对；找不到报INPUT_MISSING，不拿别的图当复现。
2. 段内p_j(q)=p_j(0)+h q Σw_l e(ψ_j(q ξ_l))，段末独立用q=1求积。h=S/N，ψ_j(q)=ψ_j(0)+h q[c0+q(c1/2+q(c2/3+q c3/4))]。修正三函数的同一数学语义，不能用整段Gauss累计权重表示q处位置。
3. 拟新增optimizer私有函数`_piece_position_samples(h,coeffs,psi0,q_values,sin,cos)`返回局部位移/ψ，可用math或CasADi标量；复用_gauss缓存，不新积分框架。保留_collocation_states现有(x,y,ψ,κ)返回及_build_casadi_nlp现有solver/bounds元组。_numeric_constraint_vector段末本来正确，只检查一致性。
4. 目标f/g的CasADi比较可在测试patch ca.nlpsol捕获NLP构造ca.Function，不为测试新造生产层。包含route/clearance各一例，并核对field边界clamp语义；不能只验证无代理目标。
5. 失败摘要优先级：已通过候选→返回；否则deadline→BUDGET_EXHAUSTED；有solver success但audit拒绝→reason AUDIT_REJECTED并保留具体拒绝；全不收敛→SOLVER_NOT_CONVERGED；全数值异常保留NUMERICAL_FAILURE。沿用status枚举和attempts，不新Result。
6. 明确solver=ipopt仅IPOPT、scipy/slsqp仅SLSQP；auto+allow且非timeout失败/剩余预算>0时每初值至多一次SLSQP。超时不回退，不改默认设置。
7. C0-B冻结同κ/η/S候选（历史未存则一次定向复现后冻结），分解Simpson余项、position_guard、angle_bound、panel数、总界。guard可能随细化增大。先修实际算错/过松界；若只能靠改5e-8到1e-7，提交同候选对比及碰撞误差包络推导后停止，待用户批准，不能自行改合同。路径1e-7预算不等于独立审核可免5e-8。

## 正反测试和量化门槛

直线所有Gauss点误差≤1e-12m；镜像/旋转平移≤1e-9。N=8/S=6的合法C2曲线，至少20个非网格点对VirtualAPath.state_at_s≤1e-7m（含其证书）；若9点Gauss不足，用固定复合细分及收敛表，不放宽阈值。相同z的numpy/CasADi f相对≤1e-9、g绝对≤1e-9。负例复现旧累计法直线配点错误；audit拒绝不得被最后solver失败覆盖；timeout不回退；fake success仍拒绝。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collocation.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_integration_bounds.py' -v
```
首文件拟新增。3组pipeline各单次，几何/动态/OBB分栏，不大规模重复。C0-B可如实得出预算仍未确认：数学分析验收与成功规划分开，相关成功声明继续阻塞。C0修复后legacy_route冻结为后续A组。Codex手算直线前缀/核同候选预算，不仅看success。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
