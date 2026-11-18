# Claude C3 返工 02：限制固定 σ 重分配的触发位置

## 结论

C3_REWORK_01 的全请求初值池修复通过复核；相邻 portal 函数能生成合法分配、通过严格 allocation 验证，且所有审核失败仍 fail closed。

但固定 σ 恢复的触发条件仍不符合任务合同：`TASK_C3.md` 与 `CLAUDE_C3_REWORK_01.md` 要求失败位于相邻 corridor 切换附近。当前 S1 失败位置不在切换相邻区间，却被重分配逻辑接受。本次只修这一项，不改初值池、不调任何审核门槛、不扩大 C3 范围。

工作区：`/home/wq/whut_thesis/expv3_multi_virtual_a`；分支 `feature/virtual-rigid-formation-a`；HEAD `9a19fd3e22d33188b13795f695757c3e4a36909f`。保护现有全部未提交改动和历史证据；不 reset/checkout/清理，不 commit/push，不进入 C4/C5。

原 C3 与 C3_REWORK_01 证据均只读。新证据目录：

`evaluation/virtual_a_corridor/C3_REWORK_02/<UTC-run-id>/`

## 1. 复现证据

S1 真实 corridor 记录位于：

`evaluation/virtual_a_corridor/C3_REWORK_01/20261117T185418Z/replays/S1_corridor_reallocation.json`

触发失败：

- containment failure `s=7.7577800095 m`，`σ=0.7640891075`，corridor index `0`；
- 原分配切换 `s=9.3543752854 m`，`σ=0.9213429943`；
- 二者相差约 `1.5966 m` / `Δσ≈0.15725`；
- 原 allocation 在切换相邻的 corridor-0 区间从 `σ=0.875` 开始；失败点 `σ=0.7641` 甚至位于更早的 corridor-0 allocation 区间。

当前 `corridor.py::reallocate_adjacent_portal_once`（约 1534–1609 行）允许只要失败点贴近 assigned corridor 的半空间边界，且旧切换本身落在两个 corridor 的交叠里，就把它认作 fixed-σ switch failure。该判断不检查失败 σ 是否靠近切换。pipeline 因而把唯一恢复用于一个不在切换邻接区间的失败点；新边界移至 `σ=0.8856843`，仍高于失败 σ，失败点依然分配给 corridor 0，二次 containment 在同一位置失败。

这没有破坏安全性，但它把 corridor 内部边界宽度失败误分类成固定 σ 分配问题，浪费唯一恢复额度，违反“仅在相邻切换附近重分配”的恢复合同。

## 2. 最小修改范围

允许修改：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/corridor.py`
- `tests/test_virtual_a_corridor_recovery.py`
- 必要时小改 `pipeline.py` 的状态记录/调用门；不得改变失败关闭逻辑。

禁止修改 C3_REWORK_01 的全请求初值计数实现；禁止修改 corridor 构造、σ 分配算法、NLP、碰撞/动态/几何审核、容差、车辆限制、ROS/RViz 和旧 FR06。

## 3. 触发语义

只有失败真实位于当前 allocation 的相邻切换邻域，才可调用 portal 重分配。采用直接、可审计的局部判据：

1. 从 allocation 找到失败 corridor `j` 与其相邻 corridor `j±1` 之间的真实切换边界 `σ_b`。
2. containment 失败 σ 必须属于该切换两侧直接相邻的 allocation 段之一；不能只因为 `point_residual_m` 接近该 corridor 某条半空间边界就触发。
3. 失败点须满足与当前失败层相符的条件：`CORRIDOR_AUDIT_WIDTH_LIMIT`，且失败点位于该相邻两 corridor 的共同可行区域内，或在一个明确由 audit `width_m` 与 `min_width_m` 推导的局部弧长邻域内。不要增加任意大距离常数。
4. 只有资格判据通过后，才用相邻两 corridor 原始半空间计算新切换点。候选新切换点仍须严格落在两 corridor 交叠内，allocation 仍由 `validate_corridor_allocation` 验证。
5. 若失败 σ 不在直接相邻 allocation 段、没有可用邻接切换或不满足局部判据，返回 `None`，pipeline 保持原有失败结果，不消耗 recovery，不再次求解。

特别地，S1 当前 `σ=0.7641` 不在旧切换 `σ=0.9213` 的直接相邻 allocation 段 `[0.875,0.9213]` 或 `[0.9213,1]` 内，因此应标记重分配不可用并直接以 `CORRIDOR_AUDIT_WIDTH_LIMIT` fail closed；不得执行第二轮 solve。

正例仍需保留：失败点位于真正切换邻接区间、两 corridor 交叠存在时，执行一次重分配；随后必须重跑 NLP 与完整审核。该正例不要求最终通过 containment。

## 4. 必须补充的测试

1. 更新 `test_failure_far_from_switch_or_degenerate_overlap_returns_none` 或新增等价测试：构造失败点虽然接近 corridor 0 半空间边界、旧 portal 本身也合法，但失败 σ 处于更早的 allocation 段；函数必须返回 `None`。
2. 保留当前 portal 正例，但将失败 σ 放在 switch 两侧直接相邻的 allocation 段，并断言只移动一个边界、allocation 连续覆盖 `[0,1]`、原始半空间和 validator 均通过。
3. pipeline 负例：注入类似 S1 的远离 switch failure，确认 `recovery_count=0`、`solve_calls=1`、不改变 allocation、无 path/plan。
4. pipeline 正例：确认合法局部固定 σ 失败恰好触发一次 recovery 和二次求解；第二次失败后无第二恢复。
5. 保留初值池测试：全请求 distinct initial `<= min(config.max_initials,6)`，duplicate launch `0`，auto fallback 仅增加 solver attempt。

## 5. 独立复验要求与门槛

运行并保存以下定向测试日志：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:$PWD:${PYTHONPATH:-}"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```

另重放一次 S1；量化证明 failure σ 是否在直接相邻 allocation 段、恢复调用次数、solve 次数、全请求初值数和最终发布资格。验收标准：远离切换的 S1 不触发 recovery；局部正例最多触发一次；所有不可执行结果均无 path/plan；原有 S3 legacy 成功和六冻结场景不回归；审核门槛不变。

保存 `SUMMARY.md`、`results.json`、`commands.log`、C3-only `source.diff`、源码哈希、测试日志、S1 重放和 start/end Git 状态。核验 C3 及 C3_REWORK_01 原证据未被修改。

返工完成后停止，等待 Codex 独立验收。不得自动进入 C4/C5，不提交、不推送。
