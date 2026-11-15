# Claude Code 任务 01：隔离基线并建立统一模式解析原语

任务 ID：FR01。版本：V0.2 / 2026-11-12。状态：**待用户确认后发出；当前不得开始实现。**

本文件可在用户确认后直接完整交给 Claude Code。完成后必须停止，由 Codex 独立验收；不得自动开始搜索/时间/ROS 或提交 Git。

## 1. 你的角色和目标

你负责本任务范围内的实现、定向测试、修复及真实结果记录。Codex 负责架构和独立验收。

**本项交付一个可独立验证的能力：构造并解析采样全车统一 A/O/S 的编队运动边，支持非零枢轴 A 协同圆弧、O 整体平移、S 独立车体调整与个别车零角保持。**

先保护当前工作区并建立隔离实施基线。不要恢复旧 T5/T6，也不要把本任务扩大为新搜索器、时序、ROS 发布或优化器开发。

必读输入（原仓库 docs）：

1. `FORMATION_REFACTOR_DECISIONS.md`：固定数学/技术合同，优先于旧 T0–T7 中冲突要求。
2. `FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md`：共同验收规则与 FR01。
3. `FORMATION_REFACTOR_REVIEW.md`：数学反例及当前接口冲突。

用户已明确：最终仅 RViz 轨迹回放，无控制/Gazebo/实车；三车统一模式、各车独立航向；所有原语边界首版可停稳；模式切换全车停稳；一般 O 转动与退化枢轴驻车首版不实现。

## 2. 工作区保护：先完成再写实现

源仓库 `/home/wq/whut_thesis/expv3_multi`，当前分支 `feat/formation-eval`，预期 HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。先检查当前事实；若 HEAD/工作树与任务描述不同，记录差异交 Codex 判断，不回滚他人工作。

源仓库有未提交 T5 三文件：

- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py`
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py`
- `tests/ros_cargo_smoke.py`

另有 `tests/test_formation_t5_frontend.py`、T5 四轮 JSON/ROS 记录、用户中文文档、历史 cargo 数据和视图工具等未跟踪内容。不能 reset/clean/stash 掉或覆盖。

### 2.1 保存证据

在源仓库新建只增不删的 `evaluation/formation_refactor_v02/baseline_<UTC>/`：

- HEAD、branch、`git status --short`、tracked diff stat 与 `git diff --binary`。
- 未跟踪/忽略的相关实验文件清单，记录路径、类型/链接目标、尺寸和关键文件 sha256。
- T5 三份源码当前内容、新测试、四轮 smoke JSON、这三份计划文档的副本与哈希；原始 ROS 日志原路径可定位，必要时本地归档，不打印其中环境认证数据。
- `BASELINE_PROVENANCE.md` 说明完整工作树与 HEAD 的区别、备份位置、后续导入文件。

快照排除自身、`.git`、build/devel/install、缓存；符号链接不追随外链；原始历史文件保留原位。记录原有 tracked diff 和关键未跟踪文件哈希，任务结束比对，源目录只允许多出本项快照。

### 2.2 创建实施 worktree

在用户确认本任务后，使用独立分支/目录：

```bash
git -C /home/wq/whut_thesis/expv3_multi worktree add \
  -b refactor/formation-v02 /home/wq/whut_thesis/expv3_multi_v02 \
  6227833011e2d92b711179bfdec979bb0b878b29
```

若目录/分支已存在，不覆盖、不重置，先核实并报告。工作区文件不因 worktree 登记而被修改，共享 Git 元数据登记属于正常创建行为。环境权限不足时申请必要权限或报告阻塞，不绕过权限。

复制三份计划、两份现状/审核文档作为冻结输入；原样导入启动脚本可能依赖的未跟踪 `tools/configure_formation_view.py`，记录来源/哈希，此项不修改该工具。不复制 build/devel 或旧 ROS 缓存，不应用 T5 patch。新代码只能在新 worktree 写入。

这不是要求立即提交分支。**本任务不执行 git commit、merge、push。**

## 3. 允许与禁止的实现范围

允许新增：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py`
- `tests/test_uniform_motion.py`
- `evaluation/formation_refactor_v02/FR01/` 的报告、机器可读结果和日志。
- 基线 provenance 文档与上述冻结副本。

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

### 5.1 A 圆弧

固定货物坐标 pivot c，`lambda∈[0,|Δpsi|]`，`a=sign(Δpsi)`，则：

```text
C_world = p_F0 + R(psi_0)c
psi(lambda) = psi_0 + a lambda
p_F(lambda) = C_world - R(psi(lambda))c
p_i(lambda) = p_F(lambda) + R(psi(lambda))r_i
rho_i = norm(r_i-c)
dp_i/dlambda = a R(psi)J(r_i-c)
v_lambda_i = sigma_i rho_i
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

本项只有空间/角度进度，没有真实时间。不得把 lambda 当秒，也不得声称已满足动态时间和碰撞审核。可以计算几何运动学和轮角；完整 OBB 与时间链在后续任务接入。

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
| P10 | S增量(π/2,0,-π/4) | 全S，包括零角车；货物/各中心固定，另两车按独立角变化 |
| P11 | S全零增量、明确±π方向 | 不生成无效零边；±π不被静默互换 |
| P12 | 反转一个direction/曲率、非法mode/ids/非有限数/无效phi | 构造或独立校验明确拒绝，不产生半有效对象 |

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

1. 新worktree路径、分支/HEAD、基线快照位置、源文件前后哈希、导入清单；源仓库T5未被修改的核对结果。
2. `evaluation/formation_refactor_v02/FR01/REPORT.md`：API、公式映射、复用/不兼容点、修改摘要、验证范围。
3. `primitive_results.json`：P01–P12实际输入、mode、半径/曲率/方向/轮角、误差、拒绝原因；逐用例通过/失败，不能只给总数。
4. 测试准确命令、stdout/stderr和退出码；完整diff（含新增文件清单）。
5. FR02需要处理的旧构造器/审核冲突，限于已确认事实，不提前实现。

遇到核心模型冲突、无法保存现状、无法建立安全worktree或现有轮系模型与解析预期冲突，停止依赖工作并给出最小复现；普通范围内缺陷自己修复。禁止放宽阈值、删除负例或用修改旧日志获得PASS。

**完成后只报告结果并停止。不继续 FR02、不恢复旧 T5/T6、不启动 ROS、不提交 Git。等待 Codex 独立验收及下一项任务。**
