# Claude C3 返工 01：全请求初值上限与固定 σ 单次恢复

## 1. 验收结论与返工边界

Codex 对 C3 首次独立验收结论为 **FAIL**。已有多路线筛选、失败关闭、真实碰撞位置局部封堵、统一软预算和三层审核继续保留；本次只修复两个阻塞问题：

1. `TASK_C3.md` 的“最多 6 初值”被实现成了“每轮最多 6 初值”，同一请求实际启动了 11 个初值；
2. `TASK_C3.md` 明确要求的“固定 σ 失败仅一次相邻 portal 范围内重分配”没有实现。

工作区固定为：

`/home/wq/whut_thesis/expv3_multi_virtual_a`

当前分支应为 `feature/virtual-rigid-formation-a`，审核时 HEAD 为 `9a19fd3e22d33188b13795f695757c3e4a36909f`。不得 reset、checkout、清理或覆盖 C0–C3、V0/V1/HUD 以及历史证据。不得提交、推送、创建分支或进入 C4/C5/V2/V3。

原 C3 证据目录必须保持不变：

`evaluation/virtual_a_corridor/C3/20261117T155212Z/`

返工证据写入：

`evaluation/virtual_a_corridor/C3_REWORK_01/<UTC-run-id>/`

## 2. 已确认的阻塞问题

### R1：6 初值上限没有作用于整个请求

`TASK_C3.md` 的合同是：最多 3 条路线、**整个请求最多 6 个初值**、最多 1 次恢复、统一 60 s 软预算。`solver=auto` 的 IPOPT→SLSQP 仍属于同一初值的求解器回退，不能被当成新的初值。

当前实现中：

- 第一轮 route-guided 启动 5 个初值；
- `endpoint_only_initial` 恢复又启动 6 个初值；
- 其中 4 个 endpoint 初值与第一轮重复；
- `S3_legacy_c3.json` 和现有测试均记录了 11 个初值/attempt。

相关代码：

- `pipeline.py` 中每轮重新调用 `solve_open_path_guided`；
- `optimizer.py::make_initial_guesses` 每轮独立产生最多 6 个初值；
- `tests/test_virtual_a_corridor_recovery.py::test_numerical_failure_spends_one_endpoint_only_recovery` 当前错误地期待 `len(calls)==11`。

这违反全请求有界搜索合同，也会用重复求解消耗 60 s 预算。报告中的“单次 solve 初值 ≤6”不能替代任务书要求。

### R2：固定 σ 失败恢复缺失

`TASK_C3.md` 明确要求：固定 σ 分配导致失败时，只允许一次相邻 portal 范围内重分配，然后重新求解和执行全部审核。

当前 `pipeline.py` 在 `audit_path_in_corridors` 返回硬失败后直接返回，没有任何重分配分支。交付报告也明确标记该能力 `NOT_IMPLEMENTED`。S1 的真实失败发生在 corridor 0/1 切换附近：

- 原切换 σ：`0.921342994303884`；
- 原因：`CORRIDOR_AUDIT_WIDTH_LIMIT`；
- 失败位置在切换附近，`point_residual_m < 0`，属于应判定是否可通过相邻 portal 重分配恢复的典型输入。

不要求 S1 最终必须成功，但必须真实执行“可重分配判定 → 至多一次新分配求解 → 完整审核”合同，并保存证据。

## 3. 允许修改范围

允许修改：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py`
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py`
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py`，仅限必要计数字段
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/corridor.py`，仅允许增加一个直接、可测试的相邻分配函数；不得改走廊构造和权威审核
- `tests/test_virtual_a_corridor_recovery.py`
- 必要时定向修改 `tests/test_virtual_a_pipeline.py`

禁止修改车辆参数、NLP 数学模型、碰撞审核、动态审核、时间律、ROS 节点、launch、RViz、旧 FR06、C2-B 验收数据和冻结场景输入。

禁止增加 manager/factory/adapter、多级恢复框架或新的全局搜索器。

## 4. R1 实施合同：全请求初值池

### 4.1 权威计数语义

增加全请求级计数，至少记录：

- `initial_count_total`：该请求实际启动过的不同初值数量；必须 `<=6`；
- `initial_count`：可继续保留为当前/最后一轮数量，但报告不得再用它证明全请求上限；
- 每轮记录 `new_initial_count`，只统计该轮首次启动的初值；
- `attempts` 继续记录求解器尝试。同一初值的 IPOPT→SLSQP 可以产生两个 attempt，但 `initial_count_total` 只增加一次。

初值身份必须由现有可重建信息确定，不使用对象地址或浮点数组哈希。建议使用现有稳定字段组合：

- route 初值：`source + route source/digest/index + label`；
- endpoint 初值：`source + length_multiplier + shape_sign + label`。

### 4.2 调度规则

1. 整个 `plan_virtual_a` 请求共享一个 `used_initial_keys` 集合和 `remaining_initial_slots=6-len(used)`。
2. 每轮只允许将尚未使用的初值交给 solver。
3. 第一轮已经尝试过的 endpoint 形状，在 endpoint-only 恢复中不得再次运行。
4. 恢复轮最多启动剩余名额；例如第一轮已启动 5 个，恢复轮最多启动 1 个。
5. 没有未使用初值时不得记录一个虚假的恢复，也不得再次调用 solver；应按原失败原因 fail closed。
6. `max_initials` 配置小于 6 时，以配置值作为全请求更严格上限。
7. 不通过修改旧默认初值、放宽约束或删除已有 route 初值来伪造计数通过。

可以在 `solve_open_path_guided` 增加一个很小的可选参数，用于传入已使用初值身份或明确的剩余初值数；也可以由 pipeline 传入已过滤的初值。选择改动更小且不会产生平行调度逻辑的方式。无论采用哪种方式，`make_initial_guesses` 仍是初值构造的唯一权威来源。

## 5. R2 实施合同：一次相邻 portal 重分配

### 5.1 触发条件

仅在以下条件全部成立时考虑该恢复：

1. `guidance_mode == corridor` 且 hard containment 未通过；
2. 当前失败确实位于两个相邻 corridor 的分配切换附近；
3. `containment.first_failure` 提供有限的 `s_m`、`corridor_index` 和残差；
4. 当前路径在相邻两个 corridor 的真实交叠范围内存在可用切换区间；
5. 本请求尚未消耗唯一一次恢复，且剩余时间和全请求初值名额均大于零。

若不满足这些条件，保持原来的 fail-closed 结果。不得把所有 containment 失败都归为固定 σ 问题。

### 5.2 最小重分配算法

在 `corridor.py` 增加一个直接函数，例如：

```python
reallocate_adjacent_portal_once(
    path, chain, allocation, containment,
) -> tuple(new_allocation) | None
```

要求：

1. 只允许移动失败处相邻 corridor `j/j+1` 的一个切换边界；其他 corridor 顺序和边界保持不变。
2. 候选新边界必须来自当前路径在 `C_j ∩ C_{j+1}` 内的连续参数区间；用两个 corridor 的原始半空间直接检查，不使用稀疏碰撞近似。
3. 新边界严格位于该交叠参数区间内部，并保持所有 allocation 区间正长、连续、覆盖 `[0,1]`、不倒序、不跳过 corridor。
4. 生成后必须调用现有 `validate_corridor_allocation`；失败返回 `None`。
5. 该函数只是恢复候选生成器，不是安全证书。新分配必须重新进入 NLP、连续 corridor containment、几何、时间、动态和 OBB 审核。
6. 不降低 `min_width_m`、深度、区间数、margin 或任何 halfspace/审核门槛。
7. 不循环扫描大量边界；只计算一次确定性候选，整个请求最多使用一次。

如果当前路径在相邻 corridor 中没有非退化交叠参数区间，记录 `fixed_sigma_reallocation_unavailable` 并保持原失败，不得扩大走廊或改变地图。

### 5.3 Pipeline 行为

hard containment 失败时：

1. 调用上述函数做一次资格判断；
2. 若得到有效新分配，消耗唯一恢复，记录 action=`adjacent_portal_reallocation`、旧/新边界、相邻 corridor、触发 reason 和失败 σ；
3. 下一轮使用同一 corridor chain/guide 和新 allocation，只运行未使用初值，且总初值仍 `<=6`；
4. 重跑全部后续审核；
5. 若再次失败，直接返回，不进行路线切换、局部封堵或第二种恢复。

## 6. 必须新增或修正的测试

### 6.1 全请求初值池

修改现有数值失败测试：

- 第一轮和恢复轮合计不同初值 `<=6`；
- 第一轮已使用 5 个时，恢复轮最多启动 1 个；
- 所有初值 key 唯一；
- `initial_count_total == 6`；
- `recovery_count == 1`；
- 失败仍无 path/plan。

增加 solver=auto 计数测试：同一初值发生 IPOPT→SLSQP 时可以有两个 attempt，但只能计为一个 initial；timeout 后仍不得启动 SLSQP。

### 6.2 相邻 portal 重分配正反例

至少包含：

1. **正例**：构造两个有非退化 portal 的 convex corridor 和一条在旧切换边界附近触发固定分配失败的路径；函数返回一个不同、合法的新 allocation。验证只移动一个相邻边界，`validate_corridor_allocation` 通过。
2. **负例**：失败点不在相邻切换附近，或路径在两个 corridor 中不存在非退化交叠参数区间；返回 `None`，不得消耗恢复、不得启动新 solver。
3. **Pipeline 调度例**：mock 第一次 containment 为可重分配失败，第二轮记录使用新 allocation；确认只恢复一次、全请求初值不超过 6，第二次失败后立即停止。
4. **安全例**：伪造重分配后 solver success，但连续 containment 或 OBB 不通过；最终必须无 path/plan。

### 6.3 现有回归

现有以下能力必须保持：

- 单盒左右路线均保留；
- 开阔狗腿去重；
- timeout 不 fallback；
- OBB 只恢复一次；
- 预算耗尽不启动新 solver；
- C2-B clearance 世界坐标语义不变；
- 旧 pipeline 六冻结场景不回归；
- 所有失败结果不携带 path/plan。

不得把当前 `len(calls)==11` 改成另一个没有身份唯一性检查的数字断言。

## 7. 必须运行的验证

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:$PWD:${PYTHONPATH:-}"

python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```

另外只需重放：

- 一个全请求初值耗尽负例；
- 一个真实 corridor 固定 σ 失败场景，优先使用 S1；
- 一个原有真实成功场景。

S1 不要求变成成功，但必须证明是否生成了合法相邻重分配、是否实际进入一次新求解，以及最终结果为何通过或失败。

## 8. 量化验收标准

- 整个请求不同初值数 `<= min(config.max_initials, 6)`；
- 重复初值启动次数为 `0`；
- recovery_count `<=1`；
- 同一初值的 solver fallback 计数语义正确；
- route 候选上限仍 `<=3`；
- 固定 σ 恢复只改一个相邻切换边界；
- 新 allocation 通过现有严格验证并完整覆盖 `[0,1]`；
- 不修改任何审核容差或车辆限制；
- 所有不可执行结果均 `path is None`、`plan is None`；
- `false_publications == 0`；
- 四条定向测试命令全部 exit 0。

## 9. 交付证据

新证据目录至少保存：

- `SUMMARY.md`：两项问题、实现、测试、仍失败场景；
- `results.json`：全请求 unique initial 数、solver attempt 数、恢复数、重分配旧/新边界及分层审核；
- `commands.log` 和完整测试日志；
- `source.diff`：仅本次返工增量；
- `source_hashes.txt`；
- 重放场景 JSON；
- start/end Git 状态；
- 原 C3 证据目录 SHA-256 未改变证明。

报告必须分别列出：路线数、不同初值数、solver attempts、恢复数、几何结果、连续 containment、动态审核、碰撞审核及发布资格。不得再用“单轮初值数”代替全请求上限。

## 10. 停止条件

完成以上返工和证据后立即停止：

- 不进入 C4/C5；
- 不运行 ROS/RViz；
- 不提交、不推送；
- 不自行声称 Codex ACCEPTED。

如果相邻 portal 重分配无法在不放宽审核的条件下形成合法 allocation，应保留负证据并停止报告，不能改成扩大 corridor、降低审核精度或增加第二次恢复。
