# C3_REWORK_02 返工证据摘要

- 合同：`plan/virtual_a_corridor_v01/CLAUDE_C3_REWORK_02.md`
- 运行目录：`evaluation/virtual_a_corridor/C3_REWORK_02/20261118T015640Z/`
- 工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- 分支 / HEAD：`feature/virtual-rigid-formation-a` / `9a19fd3e22d33188b13795f695757c3e4a36909f`
- 本阶段只修固定 σ 相邻 portal 重分配的触发条件。未改全请求初值池、corridor 构造、σ 分配算法、NLP、碰撞/动态/几何审核、容差、车辆限制、ROS/RViz；未进入 C4/C5，未 commit/push，不声称 Codex ACCEPTED。

## 1. 修复内容

### 1.1 触发前必须属于“切换直接相邻段”

`corridor.py::reallocate_adjacent_portal_once` 现在按以下顺序判定：

1. `containment.passed == false`，且 `first_failure.reason ==
   "CORRIDOR_AUDIT_WIDTH_LIMIT"`；其他失败层一律返回 `None`。
2. 从 allocation 中找真实切换：左右 entry 的 corridor index 必须连续
   （`right == left + 1`），且失败 corridor index 必须是该对之一。
3. 失败 σ **必须落在该切换两侧直接相邻的 allocation 段之一**：

   ```
   in_left  = left_segment.start  - 1e-15 <= sigma_f <= left_segment.end  + 1e-15
   in_right = right_segment.start - 1e-15 <= sigma_f <= right_segment.end + 1e-15
   ```

   不再使用 `point_residual_m` 贴近某条半空间边界作为资格条件；旧的
   `_REALLOCATION_BOUNDARY_TOL_M` 触发容差已删除。

4. 失败点必须在两条相邻 corridor 的原始半空间共同可行域内。若失败站不满足，
   只允许在由 audit 分辨率推导的局部弧长邻域内探测：

   ```
   local_radius = max(first_failure.width_m, containment.min_width_m)
   probes = s_f + {-1, -0.5, 0, +0.5, +1} * local_radius
   ```

   没有引入任意大距离常数。

5. 资格通过后才用两条 corridor 的**原始半空间**求路径参数交集，要求连续共同
   区间宽度 ≥ `1e-6 m`，取确定性中点作为新切换；候选点再次直接做双半空间
   成员检查，防止粗采样漏掉窄的 exit/re-entry 间隙。
6. 只移动该一个切换的左右 entry；结构检查与
   `validate_corridor_allocation` 必须同时通过，否则返回 `None`。

### 1.2 fail-closed 行为

任何一条资格判据失败都返回 `None`。pipeline 保持原始
`GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT`，不增加
`recovery_count`、不消耗 recovery、不启动第二轮 solver、不改变 allocation。
`pipeline.py` 只改了这一情况的 `detail` 文本：

```
no eligible adjacent allocation segment with a non-degenerate local overlap;
the fixed-sigma failure is not at a switch
```

恢复额度与求解调度逻辑未改。

## 2. 测试覆盖

`tests/test_virtual_a_corridor_recovery.py`（18 tests）：

1. **负例：远离切换的边界贴近失败**
   `test_failure_far_from_switch_or_degenerate_overlap_returns_none`
   新增 C3_REWORK_02 场景：failure `s=4.99 m`，σ=`0.62375`，点距
   corridor-0 的 y 半空间边界仅 `5e-6 m`（旧边界贴近阈值 `1e-5 m`
   本会接受）；旧 portal 在 σ=`0.75`（s=`6.0 m`）同时位于两条 corridor
   的原始半空间内，本身合法。但 σ 位于更早的 `[0.5,0.625)` allocation 段，
   而真实切换在 σ=`0.75`，直接相邻段为 `[0.625,0.75]` 与 `[0.75,1]`。
   测试先断言 failure 贴近边界、旧 switch 合法，再断言函数返回 `None`。
   同一测试还保留：局部无共同可行域、仅接触无宽度重叠、紧贴边界但距相邻
   corridor `1 m`、NaN 残差。
2. **正例：真正切换相邻**
   `test_positive_candidate_moves_one_boundary_inside_true_overlap` 两个子例：
   - 右侧段：默认链 C0 `[-1,5]`、C1 `[4,9]`，切换 σ=`0.5`；failure
     `s=4.1 m`，σ=`0.5125` 在右侧相邻段 `[0.5,1]`，共同区间 `[4,5]`，
     新边界 σ=`0.5625`。
   - 左侧段：把 C1 起点改为 `3.5`，共同区间 `[3.5,5]`；failure
     `s=3.7 m`，σ=`0.4625` 在左侧相邻段 `[0,0.5]`，新边界 σ=`0.53125`。
   - 断言只移动一个边界、allocation 连续覆盖 `[0,1]`、
     `validate_corridor_allocation` 通过、新边界点同时位于两条原始半空间。
3. **pipeline 负例（类似 S1 的远离切换失败）**
   `test_pipeline_far_from_switch_failure_does_not_spend_recovery`：
   switch σ=`0.75`，failure `s=4.99 m`/σ=`0.62375` 在更早的
   `[0.5,0.625]` 段；failure 点距 corridor-0 边界 `5e-6 m`，旧 portal
   在 σ=`0.75` 处合法。断言 `solve_calls=1`、`containment_calls=1`、
   `recovery_count=0`、`recoveries=[]`、reallocation `attempted=true` 但
   `available=false`/`consumed_recovery=false`、allocation 不变、
   `path/plan=None`。
4. **pipeline 正例**
   `test_pipeline_reallocates_once_and_stops_on_second_failure`：
   局部 failure `s=4.1 m`/σ=`0.5125` 在切换右侧相邻段；恰好一次
   `adjacent_portal_reallocation`，第二轮复用新 allocation 且只发起未用初值；
   第二次失败立即停止，无第二恢复。
5. **初值池测试保留**
   `test_route_audit_stops_before_spending_endpoint_family`、
   `test_solver_auto_fallback_is_one_initial_and_no_fake_recovery`、
   `test_numerical_failure_spends_one_endpoint_only_recovery`、
   `test_initial_pool_exhaustion_records_no_fake_recovery` 均通过。

## 3. 重放结果

### 3.1 S1（corridor，默认 `max_initials=6`）

证据：`replays/S1_corridor_reallocation.json`

- 结果：`GEOMETRY_FAILED` / `CORRIDOR_AUDIT_WIDTH_LIMIT`；无 path、无 plan、
  `executable=false`；`false_publications=0`；publication/ROS = NOT_RUN。
- failure：`s=7.7577800095459395 m`，σ=`0.7640891075134278`，
  corridor index=`0`，point residual=`-4.893289648366661e-06`，
  width=`9.68263449241391e-06 m`，`min_width_m=1e-05`。
- 原 allocation 的真实切换：σ=`0.921342994303884`
  （s=`9.354375285372408 m`），
  切换两侧直接相邻段为 `[0.875, 0.921342994303884]` 与
  `[0.921342994303884, 1.0]`。
- failure σ=`0.7640891075` 实际位于更早的 corridor-0 段
  `[0.75, 0.875, 0]`；既不在左相邻段，也不在右相邻段。
- reallocation：`attempted=true`、`available=false`、
  `consumed_recovery=false`、`new_allocation=null`、
  `status=fixed_sigma_reallocation_unavailable`，detail 为
  `no eligible adjacent allocation segment with a non-degenerate local
  overlap; the fixed-sigma failure is not at a switch`。
- recovery count：**0**；recoveries：**[]**。
- solver attempts：**1**；geometry rounds：**1**；corridor rounds：**1**
  （第一轮只发起 1 个路由初值，没有第二轮求解）。
- 全请求 distinct initial：**1**；duplicate initial launches：**0**；
  route count：**3**（≤3）。
- geometry audit：passed=true；continuous containment：failed；dynamics /
  collision audit：null（未进入后续层）。

结论：S1 的失败不满足“切换直接相邻”资格，重分配不可用，唯一 recovery 未被
消耗，也没有第二次 solve；按合同以原失败 fail closed。

### 3.2 S3（legacy route 成功回归）

证据：`replays/S3_legacy_success.json`

- `SUCCESS`；path 长度 `12.093485960786976 m`；plan 存在；`executable=true`；
  `model_binding_ok=true`。
- route count=1；distinct initial=**6**（limit 6）；duplicate=**0**；
  recovery_count=**1**，动作 `endpoint_only_initial` /
  `COLLISION_NUMERICAL_FAILURE`。
- geometry/dynamic/collision audits 全部 passed=true；`false_publications=0`。
- 与 C3_REWORK_01 的 S3 数值一致，未回归。

### 3.3 初值耗尽负例（S1，`max_initials=1`）

证据：`replays/initial_pool_exhaustion.json`

- `GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT`；无 path/plan。
- distinct initial=1；attempts=1；recovery_count=0；recoveries=[]。
- reallocation `attempted=false`、`available=false`，
  detail=`no unused initial slot remains for a reallocated solve`。
- 证明初值耗尽时不会启动第二次 solver，也不会伪造恢复。

## 4. 定向测试

完整日志在 `test_logs/`，时间与退出码在 `test_logs/run_meta.log`。

| 命令 | 结果 | 用时 |
|---|---|---|
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v` | 18 tests OK，exit 0 | 15.838 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v` | 24 tests OK，exit 0 | 5.757 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v` | 8 tests OK，exit 0 | 105.076 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v`（额外回归） | 29 tests OK，exit 0 | 65.676 s |

pipeline 套件中的六个冻结场景仍全部 SUCCESS，并保留
`initial_count_total <= min(6, max_initials)`、key 数量一致、duplicate=0、
recovery≤1 的断言；所有失败场景无 path/plan。

## 5. 变更范围

`source.diff` 仅包含相对 C3_REWORK_01 最终源码的增量
（基线由 `/tmp/c3_rework_pre` + C3_REWORK_01 `source.diff` 重建，六个哈希
逐一吻合）：

| 文件 | 插入 | 删除 |
|---|---|---|
| `corridor.py` | 80 | 79 |
| `pipeline.py` | 3 | 2 |
| `tests/test_virtual_a_corridor_recovery.py` | 198 | 29 |

- `corridor.py`：只改 `reallocate_adjacent_portal_once` 及其触发判据；
  删除仅用于旧触发逻辑的 `_REALLOCATION_BOUNDARY_TOL_M`。
- `pipeline.py`：只改 reallocation 不可用时的 `detail` 文本。
- 测试文件：只改测试。
- `optimizer.py`、`contracts.py`、`tests/test_virtual_a_pipeline.py` 与
  C3_REWORK_01 逐字节相同，未进入 diff。
- 未改审核阈值、`min_width_m`、容差、车辆参数、NLP、碰撞/动力学/时间律审核、
  corridor 构造或 σ 分配算法；未改 ROS/launch/RViz。

完整前后 SHA-256 见 `source_hashes.txt`。

## 6. 历史证据完整性

- 原 C3：`evaluation/virtual_a_corridor/C3/20261117T155212Z/`
  - `sha256sum -c manifest.sha256`：exit 0；
  - `manifest.sha256` 自身 SHA-256：
    `bcce46dfd8ce9da9ce46ae18ca54f104a6008e2a670bcd5d3ab6815096a1a466`；
  - 28 个文件；相对路径排序哈希聚合摘要
    `bb5558d4eda1083b07349d81f508eae483a982c283f25c9b2ae0214b2ee94640`。
- C3_REWORK_01：`evaluation/virtual_a_corridor/C3_REWORK_01/20261117T185418Z/`
  - `sha256sum -c manifest.sha256`：exit 0；
  - `manifest.sha256` 自身 SHA-256：
    `338e5d76e52937d1b2858986610f79f13216bf5b6ed4655feb1246d7997a3f69`；
  - 27 个文件；聚合摘要
    `e2dee6995302cb52c5b51c0a20dd02442ad5103f9893cd4d467dfea86fd5c43d`。
- 两个目录均只读访问，未写入、覆盖或移动。详见
  `original_c3_manifest_verification.log`、
  `c3_rework_01_manifest_verification.log`、
  `original_evidence_integrity_relative.log`。
- 本目录 `manifest.sha256` 覆盖除自身与 `manifest_verification.log` 外的
  全部 28 个文件，`sha256sum -c` exit 0；`manifest.sha256` 自身 SHA-256
  记录见最终交付说明。

## 7. 验收对照

`results.json` 的 `acceptance` 全部为 true：

- 远离切换的 S1 不触发 recovery（`recovery_count=0`，`recoveries=[]`）；
- S1 failure σ 不在直接相邻 allocation 段；
- S1 reallocation attempted 但 unavailable、未消耗 recovery；
- S1 只有一次 solve / 一轮 corridor containment；
- S1 无 path/plan、不可执行、false_publications=0；
- 正例/负例单测与 pipeline 正负例均通过；
- 初值池测试保留，distinct initial ≤ `min(config.max_initials, 6)`，
  duplicate=0，auto fallback 只增加 solver attempt；
- S3 legacy 成功与六个冻结场景不回归；
- optimizer/contracts/audit thresholds 未修改；
- 所有定向测试 exit 0；
- HEAD 未变、无 commit/push；
- `false_publications=0`。

## 8. 未运行与限制

- ROS、RViz、发布、真实车辆：**NOT_RUN**；本次为纯 Python 证据。
- C4/C5：**NOT_RUN**；按合同在 C3_REWORK_02 证据完成后停止。
- 未 commit、未 push、未创建分支、未 reset/checkout/clean。
- S1 仍是失败场景；本证据只证明其失败路径符合“仅在切换邻接处恢复”的合同，
  不声称 S1 必须成功。
- 不声称 Codex ACCEPTED，等待外部独立验收。