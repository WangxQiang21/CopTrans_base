# G4 rework 01：碰撞包络按侧修正与独立积分预算恢复

## 结论与范围

本报告只处理 `CLAUDE_G4_REWORK_01.md` 的 P0、P1 及必要测试和证据。G4 首次审核
`NOT ACCEPTED` 的两项事实均已修正：

- **P0**：旧 checker 的 `margin_m` 是**全宽**，实际每侧只扩张
  `margin_m/2`。修正后的 `error_envelope` 以
  `margin_used >= baseline + 2*max(e_i)` 为固定点，并用实际 binary64
  半尺寸增长验证每侧覆盖；`radius_upper` 改为在实际扩张半尺寸上向外开方。
- **P1**：参考、冻结 request、G4 支持和独立审核链恢复
  `audit_position_budget_m = 5e-8 m`。S2 的 12 m 曲率路径由新增
  `integration_bounds.py` 的**逐面板精确四阶导区间上界**确认，而不是靠高密度
  采样或放宽保护系数；G1 `1e-7 m` 生产合同、浮点/角度保护、失败关闭均不变。

没有进入 G5，没有提交 Git、建分支或 push；旧 `formation_collision.py`、旧
地图/实验、旧 G4 运行 `20261116T055008Z` 及全部 G1–G3 证据保持只读。新建运行
目录 `evaluation/virtual_formation_a/G4/20261116T081458Z`，六场景重新冻结，
manifest 自摘要为 `5bb3df2d...`，与旧运行不同。

## P0：单位语义与闭合

### 源码事实与修正

旧 `FormationCollisionChecker` 的单侧半长/半宽：

```text
half_length = (L + margin_m) / 2
half_width  = (W + margin_m) / 2
```

因此 `margin` 从基准增加 `Δm` 时，每侧增长只有 `Δm/2`。数值包络
`e_i = eps_position + (|offset_i| + r_upper)*eps_psi + r_upper*eps_relative`
是径向物理位移，必须由**每侧**增量覆盖。修正后的固定点：

```text
m = baseline + 2 * max_i e_i(m)
r_upper(m) = outward_sqrt( (half_length(m))^2 + (half_width(m))^2 )
```

实现同时校验：

```text
margin_used >= baseline
margin_used >= baseline + 2*max(e_i)
actual_per_side_growth_lower >= max(e_i)
margin_growth_lower >= pair_envelope (= e_i + e_j)
radius_upper^2 >= half_length(margin)^2 + half_width(margin)^2
```

其中 `actual_per_side_growth_lower` 由实际 float 半尺寸差向外取整，不能由
`margin_used >= baseline + e` 代替。`0.01 m` sweep padding 只记录为
`motion_sweep` 的采样保护，`padding_role` 明确它不进入数值包络、也不从
margin 扣除。

五个上下文分别记录并验证覆盖：`static_start_goal`、`no_motion_hold`、
`motion_sweep`、`map_boundary`、`vehicle_vehicle`。车辆间用两体间隙缩减
`margin_growth = margin - baseline` 覆盖 `e_i + e_j`。

### 旧 S1 正式记录重算

旧自主 S1 记录：

```text
e = 4.1494137344255645e-08 m
margin_used = 0.05000004149413735 m
(margin_used - baseline) / 2 = 2.0747068674165092e-08 m < e
```

旧记录按全宽增长 `4.1494e-08 m` 判为“覆盖”，但每侧只有其一半，**覆盖性
不成立**。这证明的是包络论证失败，不是已证明场景发生碰撞。

修正后同一车辆模型/路径误差（自主 S1）：

```text
e = 4.149413735176669e-08 m
margin_used = 0.050000082988274804 m
(margin_used - baseline) / 2 = 4.1494137403841336e-08 m >= e
closure_gap = 5.2074639302573624e-17 m
radius_upper = 0.48048002613965074 m
exact half diagonal = 0.48048002613965057 m
```

`radius_upper` 的平方以 `Fraction` 精确比较，确实不小于实际 checker 半尺寸的
平方和；它不是旧固定点半径。

### 六场景 P0 记录

下表的 `e` 和 `margin` 来自本运行 `analysis/g4_numerics.json`，`per-side`
是用记录 margin 与实际 checker 半尺寸算出的**精确每侧增量**（报告值取
binary64 结果的精确有理差值，因此可能比审计记录的保守下界略大）。

| 场景 | 结果 | e (m) | margin (m) | 每侧增量 (m) | 闭合余量 (m) | e_i+e_j (m) |
|---|---|---:|---:|---:|---:|---:|
| S1 ref | PASS | 3.0575252313398804e-08 | 0.05000006115050465 | 3.0575252329967384e-08 | 1.66e-17 | 6.11505e-08 |
| S1 | PASS | 4.149413735176669e-08 | 0.050000082988274804 | 4.1494137403841336e-08 | 5.21e-17 | 8.29883e-08 |
| S1_mirror ref | PASS | 2.996046211636168e-08 | 0.05000005992092433 | 2.9960462166744861e-08 | 5.04e-17 | 5.99209e-08 |
| S1_mirror | PASS | 3.2736834257694042e-08 | 0.050000065473668594 | 3.273683429894092e-08 | 4.12e-17 | 6.54737e-08 |
| S2 ref | PASS | 7.8549256630246153e-08 | 0.0500001570985133 | 7.854925665462531e-08 | 2.44e-17 | 1.57099e-07 |
| S2 | PASS | 7.5454102885160052e-08 | 0.050000150908205816 | 7.5454102910210707e-08 | 2.51e-17 | 1.50908e-07 |
| S2_mirror ref | PASS | 7.8549256630246127e-08 | 0.0500001570985133 | 7.854925665462531e-08 | 2.44e-17 | 1.57099e-07 |
| S2_mirror | PASS | 7.3688353536975532e-08 | 0.050000147376707167 | 7.3688353585676936e-08 | 4.87e-17 | 1.47377e-07 |
| S3 ref | PASS | 7.8549256630246153e-08 | 0.0500001570985133 | 7.854925665462531e-08 | 2.44e-17 | 1.57099e-07 |
| S3 | PASS | 4.7216468841637348e-08 | 0.050000094432937695 | 4.721646884586761e-08 | 4.23e-18 | 9.44329e-08 |
| S3_mirror ref | PASS | 7.8549256630246127e-08 | 0.0500001570985133 | 7.854925665462531e-08 | 2.44e-17 | 1.57099e-07 |
| S3_mirror | PASS | 4.4527370749449442e-08 | 0.05000008905474157 | 4.4527370790525822e-08 | 4.11e-17 | 8.90547e-08 |

所有 `margin_used >= baseline + 2*e`、每侧增量 `>= e`、两体间隙缩减
`>= e_i+e_j`、以及 `radius_upper^2 >= half_length^2+half_width^2` 均由
`g4_numerics.py` 用 `Fraction` 独立复算通过；`per-side` 正余量最小为
`4.230261209600287e-18 m`。

### 近接触反例测试

在真实旧 checker 上新增六项 P0 测试：

- `test_static_start_map_obstacle_old_margin_passes_corrected_rejects`
- `test_static_goal_map_boundary_old_margin_passes_corrected_rejects`
- `test_no_motion_hold_old_margin_passes_corrected_rejects`
- `test_motion_sweep_padding_is_not_numerical_envelope`
- `test_vehicle_pair_envelope_uses_two_body_gap_reduction`
- `test_envelope_closure_radius_and_contexts`

静态反例把障碍/地图边界放在旧侧增量未覆盖、修正侧增量覆盖的位置：
`δ = 0.75 e`，即 `e/2 < δ < e`。测试显式计算
`checker._base_half_extents`/`_distance_objects` 的半尺寸，不使用 1e-8 级
不稳定构造，并确认旧 margin 检查通过、修正后同一几何被拒绝。车辆间测试用
`VehiclePose` 和真实 `_obb_overlap`，令两体间距减少
`baseline + 1.5e` 而非修正要求的 `baseline + 2e`，验证配对包络。没有只比较
`margin >= baseline+e`，也没有只验证新旧实现输出相等。

## P1：恢复 5e-8 m 独立积分审核

### 方法

`integration_bounds.py` 对 `f(s)=exp(i ψ(s))` 精确求四阶导：

```text
k=ψ'  e=ψ''  u=ψ'''  g=ψ''''  |f|=1
f''''/f = (k^4 - 4 k u - 3 e^2) + i (g - 6 k^2 e)
```

在每段三次 Hermite 曲率多项式上，用向外区间多项式算术计算 `Re`、`Im` 的
最大绝对值，再取 `sqrt(max(Re)^2 + max(Im)^2)` 作为该 Simpson 面板的 M4
上界。系数和面板端点按文档化 `COEFFICIENT_ULPS` 向外加宽。Simpson 余项仍为

```text
sum_complete H^5 M4_panel / 2880 + one_local H^5 max_panel_M4 / 2880
```

并保留 `1e-10*(panels+8) + length*angle_bound` 的浮点/角度保护。G1 的
`1e-7 m` 生产误差合同、`1e-9 rad` 角度合同和失败关闭行为均未改变。审核
细化改为对最坏段做**等量增量**，因为 guard 随总面板数增长；余项公式、区间
上界和面板上限不变。

### 预算结果

| 场景 | 参考预算 (m) | 自主预算 (m) | 阈值 (m) | 子区间 | 细化步 |
|---|---:|---:|---:|---|---:|
| S1 | 2.9954067975737668e-08 | 3.4021948304306832e-08 | 5e-08 | 8×66 | 0 |
| S1_mirror | 2.9954067975737668e-08 | 3.2013320364665663e-08 | 5e-08 | 8×66 | 0 |
| S2 | 4.5356858422436422e-08 | 4.4168422573239431e-08 | 5e-08 | 8×76 | 0 |
| S2_mirror | 4.5356858422436422e-08 | 4.4062586419685399e-08 | 5e-08 | 8×76 | 0 |
| S3 | 4.5356858422436422e-08 | 3.7786876136816189e-08 | 5e-08 | 8×76 | 0 |
| S3_mirror | 4.5356858422436422e-08 | 3.9305947314064975e-08 | 5e-08 | 8×76 | 0 |

每条预算由 `euclidean_remainder + guard` 向外相加构成；例如 S2 参考：

```text
euclidean_remainder = 1.1436697332247596e-08
guard               = 3.3920161090188816e-08
total               = 4.5356858422436422e-08 <= 5e-08
```

`analysis/g4_numerics.py` 从序列化 knots 重建每段系数，逐面板密集采样实际
`|f''''|`（每面板 65 点），确认所有记录的面板上界都覆盖采样最大值，再用记录
面板上界复算 `full/local/segment/euclidean/guard/total` 全套算术。采样只是
佐证，区间上界才是全域依据。

新增 `tests/test_virtual_a_integration_bounds.py` 6 项测试，覆盖：S2 逐面板
界包围密集实际值、结构化三次段、直线段 M4 近零、S2 在 5e-8 下确认、不可行
阈值失败关闭、六冻结 request/support/默认值绑定 5e-8 且 G1 合同不变。

## 证据与复验入口

- 场景与参考：`scenes/scene_manifest.json`（自摘要
  `5bb3df2d829ef3b1051880fb916462cca60cd8c5a17e0c9174c8bb18370daa80`），
  六 `REFERENCE_PASSED`，`verify <scene>: PASS`，`failures=0`。
- 自主 benchmark：`benchmark/summary.json` 84/84 PASS，
  `benchmark/results.json` 6/6 executable SUCCESS，含全部尝试、三层审核、
  sweep bounds、模型与 factor-2 包络记录。
- 独立复算：`analysis/g4_numerics.json`，逐场景参考与自主的 e、margin、
  每侧增量、radius 方包围、上下文、积分预算、面板 M4 采样与精度构成。
- 测试：collision 17、integration bounds 6、pipeline 8；G1–G3 回归
  geometry 20、optimizer 26、profile 22、audit 51、timing 18、trajectory 23；
  总计 191。全部 `OK`。
- 基准：G1 26、G2 124、G3 151 检查，均 PASS；G3 9 个动态可行 timed case。
- 保护：`analysis/protection_diff_vs_g3_accepted.json` 记录 40 个新增、5 个
  授权改动、0 删除，G3 保护树
  `9bee7d7a...` 未变，当前保护树 `ccae8abae3ea...`；旧 checker、旧地图、
  旧实验和旧证据未改。
- 命令与真实退出码：`COMMANDS.md`、`logs/exit_codes.log`、
  `logs/regression_exit_codes.log`、各命令日志。
- 旧运行只读复核：`analysis/verify_historical_evidence.py` 重新哈希旧 G4
  运行 `20261116T055008Z` 自身 manifest 记录的 78 个文件（29,709,240 字节），
  全部大小与 SHA-256 一致，输出 `status=UNCHANGED`；旧 manifest SHA-256 为
  `353b1e1f8733e53a14969575ac086466edcf089ce65283d03c3f31ce886e79ef`。

## 已知问题保持真实

- G3 legacy timing 总回归仍为 FAIL：pristine HEAD `git archive` 探针退出 `1`，
  `timing_continuous` 在 `tests/test_continuous_retiming.py:271` 仍报
  `KeyError: 'dynamic_peaks'`。没有修改或放宽该旧测试，也不写成通过。
- G3 验收中记录的内部动态见证非有限值跳过
  (`_audit_check_actual_metrics`) 仍是独立非阻塞项，未声称修复。
- 本报告的 `PASS_PENDING_CODEX_RECHECK` 不是 Codex ACCEPTED；G4 是否通过由
  Codex 复验决定。

## 非声明

- 六个冻结、已知可行的 A-only 真实地图场景；不声明一般成功率、完备性、最优性
  或实时性。
- P0 反例是可控放大的合法误差/直接几何构造，用于验证覆盖语义；不证明旧场景
  实际发生碰撞，也不替代 Codex 复审。
- P1 的区间上界依赖文档化的 libm/IEEE-754 浮点假设，不是形式化浮点证明；
  65 点/面板采样只是佐证，全域结论来自记录的面板区间界。
- 没有 ROS、Gazebo、硬件或 FR06 混合模式验证。

完成后停止，等待 Codex 复审；不自动进入 G5，不提交 Git。