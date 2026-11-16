# Claude G3 返工 01：修正三车二维 jerk 的连续域上界

## 结论与执行边界

Codex 首次审核结论：**G3 NOT ACCEPTED**。已发现时间规划器和独立动态审核器在连续区间上界计算中共用同一个 `p_sss` 符号错误，可能低估车辆三阶空间导数及 jerk，故目前不能将其 `DYNAMIC_FEASIBLE` 视为可信的全域证明。点态轨迹和点态独立审核使用的该项符号正确；本问题不是已证明现有八条轨迹实际超限。

只修复本问题及必需测试、证据，不进入 G4，不修改旧 FR06、旧测试预期、物理限制，不提交 Git。工作区 `/home/wq/whut_thesis/expv3_multi_virtual_a`；分支 `feature/virtual-rigid-formation-a`；基线 HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`。保留 `evaluation/virtual_formation_a/G3/20261116T023528Z/` 等历史证据，在新唯一 run 目录记录返工。

## P0：三阶空间导数 x 分量符号错误

对车体偏移 `(a,b)`，使用编队局部坐标，`k=κ_F`、`eta=dκ_F/ds`、`u=d²κ_F/ds²`，由刚性关系可得：

```text
R(-psi) p_s   = (1-b*k, a*k)
R(-psi) p_ss  = (-b*eta-a*k², k-b*k²+a*eta)
R(-psi) p_sss = (-k²+b*k³-3*a*k*eta-b*u,
                   eta-3*b*k*eta+a*u-a*k³)
```

G1 的 `geometry.py:map_spatial_state` 和 G3 点态审核 `audit.py:_audit_spatial` 使用 `-3*a*k*eta`。但 `timing.py:_vehicle_bounds` 的 `z3` 第一分量和 `audit.py:_audit_interval_vehicle` 的同一分量使用了 **`+3*a*k*eta`**。两个上界实现相互复制，因此它们彼此一致不能证明正确。

Codex 独立数值复现（均为退化区间，给出精确点作为区间上界基本性质测试）：

```text
car1 (a,b)=(1,0), k=0.1, eta=0.1, u=0, g=0
w=1, a_time=0, j_time=0
正确 ||p_sss|| = hypot(-0.04, 0.099) = 0.10677546534667971
规划器 p_sss_norm_upper/jerk 上界 = 0.10100000000000009 / 0.10100000000000023
独立审核 jerk 上界 = 0.10100000000000023
```

两个所谓上界均低于真实值。`w=1,a_time=j_time=0` 用于检验通用区间算子的数学性质；它不是声称上述四个值必然同时由某一已交付五次时间律实例产生。该例已经足以推翻这两个函数的通用上界合同。

## 修复要求

1. 修正 `timing.py:_vehicle_bounds` 和 `audit.py:_audit_interval_vehicle` 的 `z3.x` 符号；检查完整推导及四个符号组合，保持原有限值、外扩区间与失败关闭规则。点态计算与独立审核必须同源于数学合同，但**测试 oracle 应独立实现或用数值微分/符号结果**，不能只比较两份同样的区间代码。
2. 明确动态 jerk 的区间计算 `p_sss*w³ + 3*p_ss*w*a_time + p_s*j_time` 是否覆盖每车与每个 Hermite 段的完整时间区间、节点左右极限。时间缩放阶数及默认物理限制保持原样。
3. 给出真实合法 C2 曲率路径、全路径五次时间律和三车的数值对照：在各段、节点左右及多个内部时间点，以独立点态公式或数值微分检查实际 jerk 向量模长不超过**对应局部区间证书上界**。必须覆盖前车 `a=1` 且 `k*eta` 非零、正负曲率/变化率组合及侧车；不能只看全路径最大值或比较 planner/auditor 结果一致。
4. 包含上面的退化区间直接数值反例，修复后两套计算都须给出不小于 `0.10677546534667971` 的上界（适当考虑浮点外扩）；增加窄区间覆盖，检查 z3 和 jerk 的每个维度及范数覆盖真实值。
5. 构造时间缩短或限值降低的真实负例，并记录真实车辆/时间/jerk 和审核结果：当实际 jerk 超限时不能报告 `DYNAMIC_FEASIBLE`；当上界因区间过宽无法确认时应细分或失败关闭。不得用采样最大值冒充全域证明。
6. 重跑八个 G2 冻结几何路径的 G3 时间化、NO_MOTION、151 项 benchmark 及 G3 三组测试；重计算 T、独立动态证书和模型摘要，允许实际 T 因修正变长，但不得放宽硬件限制、600 s 工程预算、积分误差或其他验收阈值。保留既有 G1/G2 测试和源码，按 G3 原任务书重跑受影响回归。

## 旧版回归记录

原始 G3 运行的未修改 baseline gate 因 workspace 白名单拒绝执行；仅扩展任务文档/fixture 白名单的 wrapper 运行中 4 项通过、`timing_continuous` 失败，失败为 `test_continuous_retiming.py:271` 的 `KeyError: 'dynamic_peaks'`。原运行在 pristine HEAD 探针复现了同一失败。返工需保留并如实报告，不能把该项写成通过；本返工不要求擅自修改旧版测试或历史证据。若需要再次执行旧版计时回归，必须使用重定向保护旧证据，并记录 gate 与 wrapper 的实际退出码。

## 交付与 Codex 复验

允许修改 `timing.py`、`audit.py` 及必要的 G3 测试、benchmark；如有其他文件必要变更，说明理由及对 G1/G2 合同的影响。交付新 `CLAUDE_REPORT.md`、精确命令/退出码、正负例数值及对应区间、8 条重新规划的时长和证书、模型/源码哈希、旧版保护差异。区分独立点态数值、连续区间证明和浮点数值假设。

Codex 将独立重算上面的数值反例，核查区间包含性和实际时间轨迹、复核 T 及独立审核、检查原有 G1/G2 与旧版保护证据，最后给出明确 ACCEPTED 或返工结论。完成后停止，等待验收和用户授权，不得自动进入 G4。
