# FR01 独立验收：需返工，暂不放行 FR02

验收人：Codex。日期：2026-11-12。状态：**CHANGES_REQUESTED**。

本结论针对当前 `uniform_motion.py`（sha256 `d75e469007172b7c0da9a706190effb8a3afc8d9fe20771391a7d528ed1726a5`）与 `test_uniform_motion.py`（sha256 `a30b740bbeedfc3d141696d93db23aa047f93cf0952c6af779bebf425938d4ba`）。没有修改实现或既有测试，没有开始 FR02，没有提交 Git。

## 1. 结论与边界

**原语主体、进度参数合同、正常构造的统一 S 轮系已获得实际验证；阻塞集中在 `validate_uniform_edge()` 的漏检。** 需要在 FR01 内补足校验与定向负例，不需要修改 V0.2 数学路线，也不需要重写搜索/时间/ROS 模块。

下文反例沿用提交测试 P10/P12 的方法：使用 `object.__setattr__` 注入构造后的错误，检查独立校验能否发现。**这些反例不表示正常工厂会生成相同错误**：工厂的构造验证已能拒绝其中多数错误。但既然模块和报告声明独立校验不信任派生字段，并已用该方法验收其能力，就不能把剩余检查完全寄托在构造器上。

## 2. 已通过的核验

【代码事实】执行目录 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。tracked/index 均干净；新增内容属于两个允许源码/测试路径与 FR01 证据目录。冻结输入、导入清单和 provenance 哈希一致。

【实验事实】原仓库仍为 `feat/formation-eval`、原 HEAD；T5 binary diff 与准备快照逐字节相同。原 tracked 文件、其他原始 untracked 文件与 T5 含忽略日志的目录均与保护清单一致。三份已批准文档修订按准备合同排除在旧版本比较之外。

【实验事实】独立复跑结果：

| 验证 | 结果 |
|---|---|
| `test_uniform_motion.py` | 12/12 通过 |
| `test_formation_model.py` | 26/26 通过 |
| `test_formation_geometry.py` | 15/15 通过 |
| `test_formation_kinematics.py` | 28/28 通过 |
| `test_continuous_retiming.py` | 18 项，1 项 error，其余通过 |
| `git diff --check` | 退出码 0；该命令不覆盖未跟踪新文件的内容 |

旧时间测试错误仍为 `tests/test_continuous_retiming.py:271` 的 `KeyError: 'dynamic_peaks'`，与 Claude 报告一致。本次是在 tracked 源码全部保持 T4 的情况下复现；该错误不作为 FR01 新增原语的返工项，不要求本项顺带修复。**时间链尚未通过完整验证，不能记为通过。**

【实验事实】Codex 另以固定随机种子 `20261112` 核对 24 条 A 圆弧：覆盖正负旋转、逐车不同前后方向、五种枢轴、非零世界位姿；使用实际 `state_at()` 的中心数值差分，而非仅对照待测函数的派生字段。结果：

- 最大独立中心误差 `8.88e-16 m`。
- 最大数值差分误差 `8.88e-10`。
- 最大沿车体横向残差 `2.89e-10`。

【代码事实/实验事实】A/O 的 m 参数与 A 弧/S 的 rad 参数、`state_at` 和一阶几何导数、正反方向与曲率关系、非零枢轴解析运动均符合本项合同。P10 正常 S 工厂中，零角车首末四轮均为规范 S：角绝对值约 `0.9357695914 rad`，因子约 `0.3068110168 m`，相对零曲率 A 有非零设置角需求。未把这些几何量当作真实速度或设置时长。

## 3. 阻塞项及复现

### R1：A 圆弧的“模式残差”没有检查车体切向约束（高优先级）

【代码事实】[uniform_motion.py:1880](../../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py#L1880) 的 A_ARC 分支比较中心导数与枢轴导数公式，但没有使用实际车体航向检查无侧滑与前后方向。`_independent_headings()` 又从输入初始航向推演期望，无法发现初始航向本身不满足 A。

【实验事实】反例 N01：从 P02 合法边出发，把全部初终车头和对应 segment 的车头同时增加 `0.2 rad`，保持角增量、曲率、方向及中心路径不变。真实横向残差为 `sin(0.2)≈0.19866933 m/rad`；校验却返回成功、`max_mode_equation_residual=0`。

【技术推断】若后续几何/航向数据生成出现相同错误，此校验无法阻止违反 A 无侧滑的边进入时间规划。当前正常工厂有切向检查，本反例针对独立审核缺口。

**要求：** 按采样处实际车头与中心导数独立检查

```text
v_lambda_i = sigma_i * ||dp_i/dlambda||
dp_i/dlambda = v_lambda_i * [cos(theta_i), sin(theta_i)]
dtheta_i/dlambda = kappa_i * v_lambda_i
```

相应残差按合同归一化；保留从几何重算的半径/曲率检查。不能只把 `__post_init__` 再调用一次，替代独立数学核对。补充正转/反转和合法倒车正例及错误车头负例。

### R2：S 末端轮系与统一模式结构没有独立核验（高优先级）

【代码事实】[uniform_motion.py:1952](../../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py#L1952) 仅调用默认 `at_end=False` 的轮系查询；独立函数没有核对全局 mode/primitive 与每车 segment.mode 的一致性。

【实验事实】以下错误全部被校验放行：

- N02：P10 零角 car2 的起点轮系为 S，但仅将末端四轮转角/因子换成 A 零曲率配置。
- N05：A_ARC 边的全局 mode 改成 S，几何与逐车 segment 仍为 A。
- N06：全局 S 边中，仅把零角 car2 的 segment.mode 改成 A。

【技术推断】N02 直接影响 FR02 基于末端轮角计算退出 S 的设置需求；N05/N06 违背全车统一模式。现有 P10 只故障注入起点轮系，未覆盖末端。

**要求：** 独立核对首末四轮的规范转角与几何因子、全局 mode/primitive/geometry_type 与逐车 segment 模式/ID；特别覆盖零角 S 的末端配置。结构和有限性检查可复用已有小型验证函数，无需新建框架；轮系几何期望继续独立按轮位置计算。

### R3：校验没有落实限值与非有限拒绝（必须随本次校验修复）

【代码事实】独立函数记录最大轮角，但未将其与 `limits.steer_angle_max` 比较；重算曲率但没有完成 `rho>=r_min_ack` 的限值拒绝。部分比较采用 `abs(actual-expected)>tol`，未预先检查有限性，NaN 会绕过比较。

【实验事实】以下错误全部被校验放行：

- N03：半径 1 m 的合法 A 弧，边限值换成 `steer_angle_max=0.1 rad`，实际最大轮角 `0.2932503135 rad`。
- N04：同一边的 `r_min_ack` 换成 2 m，各车实际半径仍为 1 m。
- N07：car1 一个起点轮角换成 NaN，最终仍返回正常有限的审核指标。

**要求：** 对审核实际消费的数据先检查有限性、合法 ID/枚举/方向及必要结构，再进行误差比较；以本边明确限值检查半径、曲率、轮角。错误须明确拒绝，不允许只返回一个超限指标等待外部调用者发现，也不得扩大容差。

## 4. 独立证据和复现命令

本轮全部输出在 [codex_acceptance_20261112T081727Z](codex_acceptance_20261112T081727Z/)：

- `baseline_verification.json`：工作区、原 T5 与输入保护核验。
- `test_runs.json` 和各 `*.log`：实际命令、环境覆盖、退出码与输出。
- `rerun_primitive_results.json`：独立复跑输出；未覆盖 Claude 的原 `primitive_results.json`。
- `independent_probes.py`、`independent_probes.json`、`independent_probes.log`：24 个正例及 N01–N07；当前 7 个非法对象均被错误接受，probe 脚本退出码 1。

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 evaluation/formation_refactor_v02/FR01/codex_acceptance_20261112T081727Z/independent_probes.py
```

该脚本故障注入只发生在进程内，不修改算法文件。返工后请将它复制到新的返工证据目录再运行，保留本次输出；也可将等价负例加入正式定向测试。

## 5. 可直接交给 Claude 的有限返工要求

> 只执行 FR01 返工 R1–R3，不进入 FR02，不改变 V0.2 模型或运动原语族。唯一工作区 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD 不变。先读本报告和冻结的 `docs/CLAUDE_TASK_01.md`。
>
> 保留已正确的 A/O/S 几何和进度参数合同。修补 `uniform_motion.py` 的独立校验：A 圆弧实际车头/无侧滑/方向与曲率关系、全局及逐车模式结构、首末规范轮系、半径/轮角限值与有限性拒绝。补充 `tests/test_uniform_motion.py` 定向负例，覆盖 N01–N07；数学残差与轮系期望独立重算，不仅重复调用构造器自证。
>
> 允许修改本项两个新增文件，并新增 `evaluation/formation_refactor_v02/FR01/rework_01/` 证据；不得修改旧算法、旧测试、配置、冻结任务文档和 Codex 本次证据。无需为此创建新的通用验证平台、依赖或额外模块。不修旧时间测试的 `dynamic_peaks` 问题，不开始时间规划/搜索/ROS。
>
> 所有原 P01–P12 正例/预期拒绝必须继续通过；N01–N07 每项均必须明确拒绝，并保留对应合法正例；零角 S 的首末规范轮系保持正确。几何/残差阈值不变，不得通过删除反例或放宽容差通过。
>
> 将新测试机器结果输出到 rework_01（使用 `FR01_RESULTS_PATH`，不要覆盖首轮记录），保存完整命令/退出码、修改摘要、哈希和新 diff，并按原样保留所有失败。复跑已使用的三个旧几何/模型定向套件即可；本项不要求扩大回归或修复旧失败。完成后停止，由 Codex 再验收，不提交 Git，不继续 FR02。

## 6. 后续准入

本次 FR01 未通过独立验收。返工通过后才讨论并授权下一阶段。时间、碰撞、设置驻留与 RViz 均未在本项验收；不因本报告对原语主体的肯定而提前宣称端到端规划可用。
