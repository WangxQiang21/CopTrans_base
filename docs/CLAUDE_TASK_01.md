# Claude Code 任务 01：统一模式解析原语（工作区已准备）

任务 ID：FR01。版本：V0.2 / 2026-11-12 第二轮审核修订。状态：**已原则批准，可由用户直接交给 Claude 执行；Codex 已完成工作区准备，尚未实现原语。**

用户将本文件交给你后，只执行这里的 FR01 实现和定向测试。完成后必须停止，由 Codex 独立验收；不得自动开始搜索/时间/ROS 或提交 Git。

## 1. 你的角色和目标

你负责本任务范围内的实现、定向测试、修复及真实结果记录。Codex 负责架构和独立验收。

**本项交付一个可独立验证的能力：构造并解析采样全车统一 A/O/S 的编队运动边，支持非零枢轴 A 协同圆弧、O 整体平移、S 独立车体调整与个别车零角保持。**

Codex 已保护原仓库并准备隔离工作区。你不得创建 worktree、备份或操作原仓库。不要恢复旧 T5/T6，也不要把本任务扩大为新搜索器、时序、ROS 发布或优化器开发。

必读输入（已复制到本工作区 docs）：

1. `FORMATION_REFACTOR_DECISIONS.md`：固定数学/技术合同，优先于旧 T0–T7 中冲突要求。
2. `FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md`：共同验收规则与 FR01。
3. `FORMATION_REFACTOR_REVIEW.md`：数学反例及当前接口冲突。

用户已明确：最终仅 RViz 轨迹回放，无控制/Gazebo/实车；三车统一模式、各车独立航向；所有原语边界首版可停稳；模式切换全车停稳；一般 O 转动与退化枢轴驻车首版不实现。

## 2. 已准备的执行环境：只核对本地，不操作原仓库

- 唯一实施目录：`/home/wq/whut_thesis/expv3_multi_v02`。
- 分支：`refactor/formation-v02`。
- 起始 HEAD：`6227833011e2d92b711179bfdec979bb0b878b29`（T4）。
- 先读本地 `BASELINE_PROVENANCE.md` 与 `evaluation/formation_refactor_v02/preparation/READY.json`、`import_manifest.json`。
- 当前准备态预期：tracked 工作树与索引干净；允许未跟踪的五份输入 Markdown、provenance、preparation 证据和原样导入的 `tools/configure_formation_view.py`。具体文件和哈希见本地清单，不把这些准备输入误认为待清理文件。

可以在本工作区运行 `git branch --show-current`、`git rev-parse HEAD`、`git status --short` 并核对本地导入清单。若路径、分支、HEAD、tracked diff 或冻结输入哈希与准备记录不符，停止报告；不切换分支、不 reset、不 stash、不清理、不重新创建 worktree。

原仓库的 T5 三份修改源码、新测试、四轮 JSON 及原始日志已由 Codex 保存并核验。新工作区没有套用 T5 patch，未复制 build/devel。你无需访问、哈希或备份原仓库，也不得修改它；不得从原仓库 devel 导入包。

**本任务不执行 git commit、merge、push 或任何工作区管理操作。**

## 3. 允许与禁止的实现范围

允许新增：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py`
- `tests/test_uniform_motion.py`
- `evaluation/formation_refactor_v02/FR01/` 的报告、机器可读结果和日志。
准备好的 provenance、导入文件与 preparation 证据只读，不属于你的修改交付范围。

若需要调整一个新模块的命名或合理拆分一个私有辅助文件，可以记录理由；不要发展成多层抽象框架。

本项现有算法/测试/ROS 配置均只读，包括 `formation_geometry.py`、`formation_kinematics.py`、Spin、S8/S9、C++、launch、run_demo、旧消息。禁止修改旧测试期待来获得通过；禁止放宽尺寸、半径、轮角或碰撞限制。现有类型无法承接合法统一 S 时，先由新边表达并记录后续适配点，不在 FR01 顺手重构共享链。

不启动 ROS/RViz、不构建 catkin、不安装新依赖、不运行全量实验或优化器。

## 4. 数据表示与 API 能力

建立不可变的 `UniformMotionEdge` 或同等明确类型，保留全局 mode、几何控制、按 vehicle_id 排序的每车初终展开航向、合法方向分支、派生曲率/斜移角以及规范起终轮系配置。中心位置始终从同一 q_F 和 offsets 重算。

优先复用：

- `formation.FormationModel/FormationPose/VehicleOffset`
- `formation_geometry.FormationPathControl` 的平移/显式 pivot 积分
- `formation_kinematics.VehicleModeSegment` 的数值载体（不调用旧混合模式枚举来替代新合同）
- `retiming.wheel_configuration` 与既有 limits
- `transport_contract.py` 和 `transport_formation.yaml`

本项不必把新边转换为旧 `FormationBlock`，因为它目前强制零角 Spin 保持车为 A。转换/共享验证修正属于 FR02。

提供的能力至少为：A 直线工厂、A 固定 pivot 圆弧工厂、O 平移工厂、全车 S 工厂、`state_at(lambda)` 和必要的一阶导数/轮系边界查询。API 签名按现有类型合理设计，但要在报告写清，不能引入另一个自由选择的逐车中心路径。

所有工厂接收实际初始车头与明确的控制/方向分支。可先有一个“生成所需 A 车头候选”的纯函数，但构造 A 边时初始车头不符必须拒绝，不能自动旋转/瞬移；搜索以后可显式插入 S。

## 5. 数学和语义要求

默认构型读取权威配置：`r1=(1,0),r2=(0,1),r3=(0,-1) m`，车体 `0.738×0.500 m`，各车中心到货物 1 m。允许各车方向不同，不允许各车 mode 不同。

### 5.0 参数域、单位与导数

每条非零边必须公开且保存 `progress_interval=[0,L]`、`progress_unit`、原语类型；L>0，lambda 从0单调增加。倒车/负旋转靠方向或角增量符号表示，不使用负长度或反向参数区间。

| 原语 | lambda / L | 单位 |
|---|---|---|
| A 直行/倒车 | 路径距离；L=共同平移距离 | m |
| A 固定pivot圆弧 | 编队旋转角进度；L=abs(Δpsi_F) | rad |
| O平移 | 平移距离；L=共同平移距离 | m |
| S自旋 | 最大车体旋转角进度；L=max_i abs(Δtheta_i) | rad |

`state_at(lambda)` 给出几何进度位置，不是时刻。导数为 `dq_F/dlambda`、`dp_i/dlambda`、`dtheta_i/dlambda`：位置导数在米制边单位 m/m、弧度制边 m/rad，角度导数相应 rad/m、rad/rad；不得称为 m/s 或 rad/s。

A/O直线：`p_F(lambda)=p_F0+lambda*d`，psi/theta固定，`dp_i/dlambda=d`、角导数0。A弧导数见下一节。S：q_F/中心导数0，`dtheta_i/dlambda=Δtheta_i/L`。实际 `lambda(t)`、车速、角速度、时间和轮系设置时长由 FR02 引入，本项禁止实现。越界或非有限进度应拒绝，不能默默吸附改变路径；全零S不伪造L。

### 5.1 A 圆弧

固定货物坐标 pivot c，`lambda∈[0,|Δpsi|]`，`a=sign(Δpsi)`，则：

```text
C_world = p_F0 + R(psi_0)c
psi(lambda) = psi_0 + a lambda
p_F(lambda) = C_world - R(psi(lambda))c
p_i(lambda) = p_F(lambda) + R(psi(lambda))r_i
rho_i = norm(r_i-c)
dp_i/dlambda = a R(psi)J(r_i-c)
v_lambda_i = sigma_i rho_i  # signed distance derivative, m/rad; NOT m/s
theta_i' = a
kappa_i = a / (sigma_i rho_i)
```

初始车头必须沿实际导数（sigma=+1）或反向（sigma=-1），theta 连续增加 a lambda。检查 `rho_i≥0.810` 和四轮配置。零半径/半径不足返回明确拒绝，不能把零半径车改成 S 混入 A。

A 直线为所有车沿同一世界位移，车头与该方向平行/反平行，曲率0。首末规范轮角/速度因子应来自实际每车曲率/模式。

### 5.2 O 平移

psi 和每车 theta 均固定，同一位移方向 d：

```text
d = sigma_i * [cos(theta_i+phi_i), sin(theta_i+phi_i)]
abs(phi_i) <= pi/2
```

phi 恒定，逐车可以不同。检查完整向量和方向，不靠沿车头投影判断 ±90° 横移的方向。禁止加入 O 转动编队。

### 5.3 S

q_F/中心固定，全车 S。明确 Δtheta_i∈[-π,π]，L=max(abs(Δtheta_i))，theta_i=theta_i0+Δtheta_i lambda/L。允许Δtheta_i=0，且该车仍S；整个Δ均0时返回不生成边的明确结果，不伪造L或自环。恰为±π的旋转方向需明确保留，不静默反转。

**零角车辆的轮系仍必须为S。** 每辆车的首末轮角及单位车体自旋速率因子，都应匹配 `wheel_configuration(Action(SPIN,0), limits)`。不能将零角车只标S却保留 A/零曲率配置。

独立期望：轮位置 `(x_w,y_w)=(±wheelbase/2,±track/2)`，单位车体自旋速度为 `(-y_w,x_w)`；轮角 `atan2(x_w,-y_w)` 按现有规范折入 `[-π/2,π/2]`，因子幅值 `sqrt(x_w²+y_w²)`。零角车的实际车体/轮速为0，但规范S轮角和该几何因子不因此退化成A配置或被清零。

必须输出首末S轮系，并核对零角车与旋转车使用相同的S规范（相同尺寸时）；报告相对零曲率A配置的角差，证明 FR02 不能因为Δtheta_i=0就省略该车进入/退出S的设置。这里只比较角差和规范配置，不计算设置时长或实际轮速。

本项只有空间/角度进度，没有真实时间。不得把 lambda 当秒，也不得声称已满足动态时间和碰撞审核。可以计算几何运动学、规范轮角和速率因子；完整 OBB 与时间链在后续任务接入。

## 6. 必须覆盖的解析用例

独立测试计算期望值，不仅调用同一构造器再断言其 success。固定随机种子和少量非均匀进度点检查变换/刚性即可，无需大规模采样。

| 编号 | 用例 | 必须验证 |
|---|---|---|
| P01 | A 直行，三车车头(0,π,0)，允许倒车 | 所有A，方向(+,-,+)，曲率0，刚性、车头不跳变 |
| P02 | c=0，正转，切向车头(π/2,π,0) | rho均1、kappa均+1，终端车头与货物增量正确 |
| P03 | 保持P02初始车头，改顺时针且允许倒车 | direction均-1、kappa均+1；不能重现旧“标正向但实际反向”错误 |
| P04 | c=(0,2)，正转 | rho=(√5,1,3)，相对车头=(atan2(1,2),0,0)，曲率=(1/√5,1,1/3) |
| P05 | P04从非零psi_0=.37开始，转π/2 | 世界中心/货物弧正确，不能忘记旋转偏移或pivot |
| P06 | c=(0,1.5) | car2半径.5，明确拒绝；货物半径1.5合格不能代替逐车检查 |
| P07 | pivot恰在car1中心 | 明确“首版退化分支不支持”，无除零/NaN，不宣称一般物理不可能 |
| P08 | 给c=0的A弧输入三车车头全0 | 拒绝/报告需S对齐；不偷偷改车头 |
| P09 | O横移/斜移，含±90°与不同初始theta | 全O，phi/方向向量正确，psi/theta固定 |
| P10 | S增量(π/2,0,-π/4) | 全S；货物/中心固定，独立角变化；零角车首末四轮均为规范S角/因子，不能为A配置；进入/退出设置所需配置完整保留 |
| P11 | S全零增量、明确±π方向 | 不生成无效零边；±π不被静默互换 |
| P12 | 反转一个direction/曲率、非法mode/ids/非有限数/无效phi | 构造或独立校验明确拒绝，不产生半有效对象 |

P10还必须有“只改mode=S、轮系仍是A”的篡改负例并被拒绝；不能只用待测工厂调用同一helper自证，须按轮位置独立重算规范角与因子。相对A零曲率的非零转角需求以rad记录，不计秒。

所有类型都检查 progress_interval/unit；特别验证A弧的lambda不是货物/车辆弧长、S的lambda不是某个指定车的角度或秒。

对P02–P05用中心数值差分核对沿车头投影及侧向残差；对A用theta'/signed speed核对曲率；对O用实际d与sigma/phi完整向量比较。独立微分不得直接拿派生方向字段作为“真实速度”。

## 7. 验收阈值和命令

- 解析中心/末端几何误差≤1e-7 m；刚性残差≤1e-6 m。
- 非零运动归一化模式方程残差≤1e-6；独立有限差分（远离端点/零速）≤1e-4。
- 曲率、轮角不得超限；方向、vehicle_id和全车模式必须完全一致于合同。
- 预期拒绝用例必须明确拒绝；运行不能出现意外失败/错误、NaN、方向隐式跳变。

在新worktree下运行本项纯Python测试，测试须明确将新worktree的包路径插入sys.path，不借原仓库devel导入：

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_uniform_motion.py' -v
git diff --check
```

可运行与本项实际使用相关的既有几何/模型定向测试；无需完整427项回归。记录解释器、包文件实际路径和依赖版本；缺依赖先报告，不未经授权安装或借错overlay“调通”。

## 8. 交付物和停止条件

提交给 Codex 审阅的材料：

1. 本地worktree路径、分支/HEAD、开始/结束Git状态及冻结输入校验结果；引用 Codex 已准备的 provenance，不访问原仓库做重复保护。
2. `evaluation/formation_refactor_v02/FR01/REPORT.md`：API、公式映射、复用/不兼容点、修改摘要、验证范围。
3. `primitive_results.json`：P01–P12实际输入、mode、进度区间/单位、导数含义、半径/曲率/方向/首末轮角与因子、误差、拒绝原因；逐用例通过/失败，不能只给总数。
4. 测试准确命令、stdout/stderr和退出码；完整diff（含新增文件清单）。
5. FR02需要处理的旧构造器/审核冲突，限于已确认事实，不提前实现。

遇到本地准备态不符、核心模型冲突或现有轮系模型与解析预期冲突，停止依赖工作并给出最小复现；不得转去操作原仓库或重建worktree。普通范围内缺陷自己修复。禁止放宽阈值、删除负例或用修改旧日志获得PASS。

**完成后只报告结果并停止。不继续 FR02、不恢复旧 T5/T6、不启动 ROS、不提交 Git。等待 Codex 独立验收及下一项任务。**
