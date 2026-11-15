# Claude Code 任务 02：公共时间、权威解析轨迹与独立审核

任务：FR02。版本：V0.2 / 2026-11-13。执行方式：**先完成 FR02-A 并停止，经 Codex 独立验收 ACCEPTED 后，获得用户下一步指令才执行 FR02-B。** 本文件不授权连续执行两个验收点。

本文件交给 Claude 后，只在已经准备的 `/home/wq/whut_thesis/expv3_multi_v02` 实现和定向测试。Codex 负责独立验收。不操作原仓库、不创建 worktree、不提交 Git。不得开展 FR03 搜索、FR05 ROS/RViz、消息定义、NLP、控制或动力学任务。

## 1. 输入、基线与交付目标

先读：

1. `docs/FORMATION_REFACTOR_DECISIONS.md`，尤其第5、6、8节。
2. `docs/FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md` 的 FR02 和共同验收标准；本文件将其分成两个顺序验收点。
3. `evaluation/formation_refactor_v02/FR01/CODEX_ACCEPTANCE.json` 及其 detailed_report。
4. `BASELINE_PROVENANCE.md`、`evaluation/formation_refactor_v02/preparation/`，以及 FR01 实际源码。

固定工作区分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。FR01 当前验收为 ACCEPTED；原先 `FR02_execution_authorized=false` 是上一轮未启动下一阶段的历史记录，不是本任务交付后的禁止准入条件。不要改写这条历史记录来“启用”任务。

FR01 两个文件只读，启动与结束都核对 sha256：

| 文件（包路径基准见下节） | 已验收 sha256 |
|---|---|
| `uniform_motion.py` | `1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2` |
| `tests/test_uniform_motion.py` | `7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a` |

记录本地完整 Git 状态、输入清单和实际导入路径；tracked 初始干净，FR01 与准备文件未跟踪属于已知基线。逐项保存本次要修改的共享文件的初始副本/哈希，最终 diff 必须包含未跟踪文件，不用 `git diff` 为空证明没有改动。发现已有未知修改、HEAD/输入哈希不符则停止报告，不覆盖、不 reset/stash/clean。原仓库 T5 与历史输出不在你的操作范围。

最终能力：对**人工指定的合法 UniformMotionEdge 序列**产生公共相位时间表、可任意时刻求值的权威解析模型及其采样投影，完成独立动力学/轮系/碰撞/一致性审核。不得重新挑选模式、方向、车头或几何路线。

## 2. 已有接口与局部适配方案

本文件中包路径 `P/` 指 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。下表“已有”是实际接口，“新增”是本任务要求建立的能力，不能声称它们已经存在。

| 已有能力 | 实际接口及复用要求 |
|---|---|
| FR01 权威边 | `P/uniform_motion.py`：四个工厂 `a_straight_edge/a_arc_edge/o_translation_edge/s_spin_edge`；`UniformMotionEdge` 保存 model/control、初终展开车头、directions、curvatures/oblique_angles、signed_progress_speeds、首末轮角/因子、segments、limits/allow_reverse |
| 几何求值 | `state_at(progress)` 返回 qf/centers/headings；`derivative_at(progress)` 返回 dqf/dp/dtheta 及单位；`progress_interval/progress_unit`、`wheel_configurations(at_end=...)`、`validate_uniform_edge()` 已存在 |
| S8 | `P/formation_retiming.py`：`FormationTimingLimits`、`_motion_limits`、`_context_for`、`_analyze_block`、`_peak_table`、`_quantities`、`retime_formation_plan`、`audit_formation_timing`；复用动态时长计算与诊断，不能复制一套峰值求解系统 |
| 旧几何载体 | `P/formation_geometry.py`：`FormationBlock`、`FormationContinuousPlan`；从 FR01 control/segments 构造审计/分析视图，不能借旧构造器重新枚举模式或拟合几何 |
| 设置计时 | `P/mode_switch_timing.py:mode_switch_event(delta, limits)`；`P/retiming.py:steering_rest_to_rest_duration`、规范轮系工具；复用 restricted 规则，不用单车交互配置提速 |
| S9 | `P/formation_quality.py:evaluate_formation_trajectory(snapshot, plan, timed, grid, config=None)` 和 `_IndependentEvaluator`；入口目前强制旧 Plan/Timed 类型；扩展现有审核链，不抄出第二个完整 evaluator |
| 碰撞 | `P/formation_collision.py:FormationCollisionChecker.static_check/swept_check`、`FormationState`、`SweepPolicy`；保留 OBB、距离、车辆间及地图碰撞和保守 padding |
| 零运动 | `P/formation_hold.py:build_hold_timed_trajectory/evaluate_formation_hold`；现有零时长单采样保持可复用，V0.2 解析模型仍须有明确静止状态 |

### 已确认的不兼容与最小处理

1. FR01 **没有** `from_dict`、时间域求值或二/三阶导数 API。`as_dict()` 是诊断输出，不能直接当成已验证的可逆序列化协议。新增一个最小版本化模型编码/解码层；通过 FR01 工厂重建边，所有冗余字段重新计算/比对。高阶导数从固定枢轴/平移/S 的闭式公式计算，复用 S8/S9 已有导数逻辑；不修改 FR01 来追加功能。
2. `FormationBlock`、`formation_kinematics.audit_mode_assignment`、S8 `_context_for`、S9 `_check_block_kinematics` 有零角 S 拒绝。局部修正为：中心固定、控制0、行程0、车头增量0且轮系为规范 S 时允许该 S 保持；非零 S 的方向检查继续保留。旧合法混合构型可以继续存在，但 V0.2 入口另行强制统一模式；不得把零角车改回 A 来适配。
3. S8 `_quantities/_sample_motion`、S9 `_dynamic_values/_check_samples` 的旧 `speed_signed` 是车体纵向投影。V0.2 纯 O 横移的行驶速度不能直接取它。保留旧语义，通过明确版本化 V0.2 入口计算 `sigma*||p_dot||`，不全局静默换义。
4. 旧 S8 完整相位/采样输出不具备本任务全部初始设置和解析协议，故优先调用 `_analyze_block` 获取运动时长，再由薄适配器组织公共相位。不要把 `retime_formation_plan` 的结果直接贴 V0.2 标签。S9 当前也不能直接接受新模型；在原审核链增加显式、严格校验的解析模型上下文，默认旧入口行为保持，不用“跳过旧审核”的开关绕过检查。
5. 旧 `swept_check` 会在状态端点之间线性插值 q_F。对非零枢轴弧，这不是原弧。FR02-B 在同一碰撞模块增加最小解析采样入口/可选采样器，内部复用原碰撞内核和保守覆盖；不能只加密线性插值后称两条轨迹严格一致。

若局部扩展无法承接合法输入，提交具体调用链与最小反例，停止相关工作；不先发展平行 retimer/evaluator，也不修改 FR01 数学模型。

## 3. 共同数学合同

### 3.1 几何与公共时间

全程 `p_i=p_F+R(psi_F)r_i`；各车 theta 独立、全车模式统一。边接续须核对 vehicle IDs、offsets、几何限值、q_F 与**展开车头**；设置相位不能修正位置/车头不连续。角度接续不以随意加减2π改变 S 旋转分支。

每条边 λ∈[0,L]：A直行/O为m；A固定枢轴弧为编队旋转角rad；S为最大车体角进度rad。公共运动相位 t∈[t0,t0+T]：

```text
tau=(t-t0)/T
s(tau)=10*tau^3-15*tau^4+6*tau^5
lambda=L*s(tau)
p_dot=p' lambda_dot
p_ddot=p'' lambda_dot^2+p' lambda_ddot
p_dddot=p''' lambda_dot^3+3 p'' lambda_dot lambda_ddot+p' lambda_dddot
theta_dot=theta' lambda_dot
theta_ddot=theta' lambda_ddot; theta_dddot=theta' lambda_dddot
```

本原语的 theta' 为常数。平移 p''/p''' 为0，S 中心导数均0；A弧令 a=sign(Δpsi)，p_i'=a R J(r_i-c)、p_i''=-R(r_i-c)、p_i'''=-a R J(r_i-c)，货物用 r=0。不得用任意 dt 的折线差分替代动态限值计算。

各车使用同一个 T：优先复用 S8 对全部车辆、全部约束的单位时长峰值与最大时长规则。速度、加速度、jerk、yaw及轮速、横向加速度均受 `FormationTimingLimits` 限制，保留其安全系数/驻留/120s上限等实际默认值与失败诊断。FR01 MotionLimits 与 TimingLimits 的 wheelbase/track/r_min/轮角必须一致，不静默取更宽者；不一致明确拒绝。不能以三条独立轨迹重采样来同步。

首版每条边端点线/角速度及进度加速度为0；同模式停车属于首版简化。T内几何边不变，不能插入未声明运动。A/O `center_speed_mps=sigma||p_dot||`，S/setup为0；车体纵向投影单独诊断。yaw_rate保留符号；轮速限值使用绝对值，S零角车实际轮速0而规范轮系因子非0。

### 3.2 初始设置、公共驻留与模式边界

- 起始明确为全车静止 A、零曲率规范轮系。输入车头原样保留；首边不匹配时不能用设置驻留偷偷自旋。
- 初始到首边、任意相邻边，均逐车逐轮比较实际规范轮角。模式变或轮角变才产生 setup；同模式同配置边可直接接续停车边界，不造零时长相位。
- 模式变更逐车调用 `mode_switch_event`，restricted profile；公共设置时长取最慢车辆需求。相同模式轮角变化复用已有转向计时规则；固定0.3s不能覆盖更长实际需求。记录每车请求、公共总时长和轮角差。
- 用统一公共转向曲线实现同步：先保留公共 setup 开销，再在公共可用转向时间内以五次曲线从各轮起角到终角；公共时间不得短于每车事件要求。记录该公共曲线实际轮角速度/加速度并核验，而非直接沿用各车更短 profile 的峰值。
- setup 全车 p_F/psi/centers/theta 固定、车体线/角速度与加速度0；轮子可重配置。setup不是S自旋；设置期间轮角不必是任一纯模式的规范配置，须标记 transition/from_mode/to_mode。
- 所有相位采用左闭右开，setup期间标签为旧模式，结束时全车同时进入新模式；t=T_end另定义最终静止状态。同模式重配置标签不变。采样网格须含0、所有边界、T_end，每时刻每车仅一个状态。
- 首次需要设置必须有 initial_setup；若首边A零曲率则初始设置时长0、记录为无需设置，不造空相位。终点不额外退出S或回A。
- `min_mode_dwell_s` 复用现有S8语义及默认值，不能降低为0；记录其如何作用于运动相位，独立审核同样核对。正时长相位无时间缝隙/重叠，T_end为各相位时长总和。

### 3.3 权威解析模型与序列化（本项新增，接口名可合理调整）

建议 `UniformTimedPlan.state_at(t)`、`to_model_dict/from_model_dict` 和 `sample(times)`；这些是**待实现接口**。state包含公共q_F、逐车中心/展开车头、全局模式、相位类型、λ/单位、带符号速度/yaw、轮角及设置状态。通过一个规范求值器生成所有采样和未来回放状态；不独立插值三车中心，也不线性插值弧上的货物位置。

编码保存版本/plan_id/formation_id、offsets/形状与limits、初始q_F/车头/模式轮系、每条边的原始工厂参数/方向、相位时长及设置需求。不要序列化Python可执行对象。拒绝未知版本、非有限数、缺失/重复ID、非法枚举、冗余值冲突和接续失败。反序列化不重新定时而改变输入相位；它严格还原且校验，FR02-B再独立判定时长可行性。摘要可用但不能代替实值校验。

`state_at(t)` 对[0,T_end]任意有限时间给出精确解析状态；越界明确拒绝，未来播放器若需要终点钳位在外部处理。空边序列只在显式起终点q_F与车头相同的零运动请求合法，输出 duration=0、单采样静止A，与现有hold能力对齐；不虚构S边或ε时长。相同位置但车头不同属于S运动，不是hold。

采样投影可使用现有 `FormationTimedPoint/FormationTimedTrajectory` 数值载体，必须显式标记V0.2语义与版本，不能被旧入口无版本解释；它不成为第二条权威轨迹。没有审核通过前不得标记为可发布/安全认证。

## 4. FR02-A：公共时间与解析相位（第一个验收点）

### 目标、范围与文件

只接受人工边序列，交付公共时间、initial/setup/motion相位、任意时刻解析求值、版本化往返与投影。不宣称碰撞通过，也不实现完整S9审核。

允许新增：

- `P/uniform_timing.py`：FR01→旧S8分析视图、复用时长、设置调度的薄适配。
- `P/uniform_trajectory.py`：一个权威解析时间模型、求值/编码/投影。
- `tests/test_uniform_timing.py`、`tests/test_uniform_trajectory.py`。
- `evaluation/formation_refactor_v02/FR02/A/` 证据。

仅为合法零角S支持与S8适配，允许定点修改 `P/formation_geometry.py`、`P/formation_kinematics.py`、`P/formation_retiming.py`。逐函数解释必要性；不动旧模式枚举/路线生成/优化逻辑。`retiming.py`、`mode_switch_timing.py`、hold、碰撞、S9和FR01本验收点只读。调用私有S8 helper时集中在一个薄适配边界并测试实参类型，不散落复制其实现。

### 必须测试

| ID | 正例/反例与判定 |
|---|---|
| A01 | A零曲率首段：无伪初始设置；首段O横移/S：有真实初始设置，固定车头，全车同步 |
| A02 | S增量(π/2,0,-π/4)：三车全S，零角car2首末规范S；公共T受最大旋转车辆约束 |
| A03 | S显式对齐→c=(0,2)的A弧（非零初始psi）→S→O；接续按工厂实际所需车头构造，不能直接连接不匹配的A/O来凑场景 |
| A04 | 两条接续且同模式的不同合法控制：有轮角差则有设置；同模式同配置则无设置。允许用固定车头O的不同位移方向构造，所有边仍停车 |
| A05 | O的±90°横移与合法反向：真实中心速非零，带符号行驶速度正确；纵向投影接近0不能代替行驶速度 |
| A06 | λ为rad的A弧和S、λ为m的直线；五次链式法则与独立闭式/中心差分吻合；分别受速度/加速度/yaw/轮速等驱动的手工场景，不依赖单一限制 |
| A07 | setup内部/两侧边界：旧标签保持到结束，三车同时切换，wheel rate/accel受限；初始设置和S运动分项计时不重计 |
| A08 | 模型往返后，在节点、边界两侧和每运动/设置相位≥11个非均匀内部时间求值一致；非零pivot弧内部q_F必须在解析弧上 |
| A09 | 真零运动保持单采样；位置相同但车头不同不得误判hold；起终请求未满足不能把空序列当成功 |
| A10 | 接续跳变、IDs/offsets/几何限值不一致、非法版本/非有限输入、非法进度/时间、零/负时长、时间缝隙/重叠均明确拒绝；T超上限返回真实限制原因 |

### 数值、回归与证据

中心/q_F几何与接续误差≤1e-7m；展开车头接续/往返误差≤1e-7rad；刚性≤1e-6m；归一化模式残差≤1e-6；独立有限差分误差≤1e-4（离开端点/零速）；时轴接续误差≤1e-9s；端点线/角速度及进度加速度绝对值≤1e-9（各自SI单位）。所有动态峰值≤本边限值×(1+1e-3)，旧检查更严格时遵从旧标准。设置段车体pose误差≤1e-9m/rad。不能放宽车辆/扫掠限值。

定向运行新两个测试、FR01 13项，以及 `test_formation_geometry.py`、`test_formation_kinematics.py`、`test_formation_retiming.py`；若使用hold则加`test_formation_hold.py`。只读旧测试，不改预期。已知 `test_continuous_retiming.py:271 dynamic_peaks` 为另一旧套件错误，不计通过，不强制顺带修复。旧测试如与零角S新合同确有直接冲突，保存具体用例并停报，不自行改旧测试期待。

`FR02/A/`必须保存：REPORT.md、机器结果逐A01–A10、真实命令/环境/退出码/完整输出、模型JSON与投影、初始状态/limits、逐车设置需求与公共相位表、峰值与驱动约束、非零pivot弧内部样本、序列化往返误差、失败输入与原因、共享文件前后副本/哈希及全部diff。每条测试导入路径来自本worktree，不借原仓库devel；FR01_RESULTS_PATH指向本轮目录，避免覆盖旧结果。

**完成A即停止，交Codex验收。A没有碰撞/S9证书，不因时间模型可用而继续B。**

## 5. FR02-B：独立审核、真实扫掠与模型/投影一致性

### 准入、范围与文件

仅在A有Codex ACCEPTED记录且用户明确交付B后开始。先保存A的源码/证据哈希。默认A两个新模块只读；B如发现其真实缺陷，给最小复现并停止依赖验收，由Codex判断回退修复A，不静默改动已验收合同。

允许新增 `P/uniform_quality.py` 薄审核入口、`tests/test_uniform_quality.py` 和 `FR02/B/` 证据；允许定点扩展 `P/formation_quality.py`、`P/formation_collision.py`，以必要的零角S修正、显式解析上下文与V0.2速度语义复用S9原链及碰撞内核。不改变旧默认入口、旧消息/ROS/发布门控。A的共享文件此阶段默认只读，不扩大修改清单。

审核入口输入原请求、权威模型、采样投影、实际地图/形状/限值；不能只接收求值器的success。具体函数名在A报告中约定，属于本项新增API。

### 独立审核与碰撞要求

- 独立从几何与五次时间推导中心、速度/加速度/jerk、车体yaw、轮速及横向加速度；复用现有S9 `_dynamic_values` 等独立计算核，不调用S8的峰值报告代替审核，不单凭FR01校验成功或JSON摘要作为证明。
- 检查每车全程统一mode、body-spin零角保持、合法方向/曲率/O完整向量、规范首末轮系、实际setup轮角曲线和最慢设置下界；缩短公共时间、漏初始设置或漏零角S车需求必须被拒绝。规范切换和setup中间轮系区分检查。
- 检查所有相位边界及两侧、模型节点、投影时刻和每相位≥11个内部时刻。精确model为唯一运动依据；投影的q_F/中心/theta/模式/带符号速度/yaw/相位/轮系字段（存在的字段）逐项比较。不得从投影点列插值反过来定义轨迹。单纯dt加密不是正确性证明。
- 动态审核不能仅凭11个样本证明极值：沿用S9/S8已有峰值候选/解析界方法；补足setup转向曲线极值。若数值候选缺覆盖则明确限制并给保守上界，不擅自宣称连续时间严格证书。
- 在现有碰撞检查器中以真实解析状态采样，保留地图/车间OBB、静态边界、Spin完整车体扫掠和原安全margin。推荐按每条边λ细分，使用角点位移界

```text
B_i <= (sup ||p_i'(lambda)|| + body_half_diagonal_i * sup |theta_i'|) * Δlambda
```

使每段B≤`SweepPolicy.max_corner_step_m`，在精确采样点使用其原`motion_padding_m`覆盖段内运动。A弧/S/平移的导数范数为常数，可直接给保守界；setup位姿固定。不得先线性插值q_F再验碰；若复用旧swept_check的外壳，必须保证其新增V0.2路径确实在解析模型取中间状态，而非仍调用旧`_interpolate`。保留样本预算与预算失败，不能越界减margin/步长以通过。
- S9返回的审核/碰撞结果必须关联同一模型、地图签名与参数。失败不得提供成功证书/可用投影来替代失败。此次不实现ROS publication gate或在线状态机。

### 必须测试、标准与证据

| ID | 正例及故障注入 |
|---|---|
| B01 | A全部合法手工场景、hold和S零角保持：完整S9/碰撞通过，模型往返前后结论一致 |
| B02 | 把A车头整体偏离切线但保留中心路；篡改方向/曲率/O速度投影；各自明确拒绝 |
| B03 | 全局/逐车mode冲突、S零角车首端或末端改A轮系、篡改规范因子/ID/NaN：拒绝；同时保留合法正例 |
| B04 | 缩短运动时间使速度/加速度等超限、缩短或删初始/相邻setup、漏最慢车辆需求、时轴重叠/缝隙、提前切mode：拒绝并指出相位/车辆/约束 |
| B05 | 只改投影中心/货物位置/展开车头/时间/模式/速度/yaw，模型不变：一致性审核拒绝；不能因为单改后仍满足刚性而放行 |
| B06 | S起终OBB均无碰撞但中途车体扫掠碰撞：拒绝。非零pivot A弧中途碰撞：拒绝；保存障碍、碰撞时刻与解析状态，不能仅检测端点 |
| B07 | 固定同一模型，分别以0.02/0.05s生成投影，精确同一组不在网格上的时刻状态一致；篡改成沿货物首末弦插值的结果须被拒绝；不得强求新旧离散净空数值完全相等 |
| B08 | 未知模型版本/非法地图签名、投影与模型身份不符、扫掠预算耗尽：明确失败，不能冒充运动无解或审核通过 |

数值遵从A标准与现有S9更严格阈值。必须给出内部时刻几何误差、刚性、模式残差、动态利用率、首末轮系误差、setup下界/实际时间和碰撞覆盖界/样本数量；不得用“所有指标小于阈值”的一句话替代原始机器结果。

运行FR01、A新测试、B新测试，以及 `test_formation_quality.py`、`test_formation_collision.py`；若使用/触及hold相关路径则加 `test_formation_hold.py`。改共享模块影响其他已知定向套件时追加对应套件，不运行大规模地图实验。保持原失败记录，不改旧预期。

`FR02/B/`保存与A相同级别的命令/退出码/版本与diff证据，另加：独立审核JSON、每条篡改前后输入与拒绝位置、动态峰值候选/界、扫掠策略与覆盖证明/预算、碰撞地图与首次碰撞状态、投影一致性结果及不同采样率比较。所有成功/失败模型和限值可重现，不能仅保留截图或总通过数。

**完成B后停止，等待Codex独立验收。不进入FR03、不接ROS、不提交Git。**

## 6. 通用停止与禁止事项

不得改FR01两个已验收文件、冻结计划/决策、旧测试期待、车辆尺寸/offsets/r_min/轮角/轮速/安全阈值、旧实验与Codex证据。不得隐藏运行失败、覆盖历史结果、关闭某类审核或用legacy可行冒充V0.2统一模式可行。允许明确的版本化V0.2语义扩展，不允许以版本作为跳过动力学或碰撞的通行证。

普通范围内工程缺陷自行修复；需要改变模式语义、运动原语/刚性模型、时间合同，或需要超出文件清单改动时，给出最小反例和最小建议并停止依赖工作，由用户/Codex决策。不得擅自改数学路线。

每个验收点交付实现摘要、真实测试结果、未验证项、完整diff与证据目录，**只执行当前获准的一个验收点**。首轮默认只执行FR02-A。
