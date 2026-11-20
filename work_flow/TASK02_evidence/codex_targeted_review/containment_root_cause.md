# 连续 corridor containment 归因

## 本次两个用户 plan

**NOT_REPRODUCIBLE_FROM_AVAILABLE_INPUT。** 未取得 plan 600001/600002 对应的 frozen request/map/context identity，因此不把其他同号请求的 failure 归因到这两条计划。

## 最近 TASK02 S1 证据（不同输入）

`work_flow/TASK02_evidence/20261119T054527Z/rework/live/s1_default/quality_report.json`，plan 600000，request `591b4100…`，grid `6e94c751…`，context `8a545c3f…`。

- route 1 / solver round 0：连续 F containment PASS，动态审核 PASS，最终 OBB sweep FAIL：`MAP_OBB_COLLISION`，vehicle 2，station 2.654537 m，time 10.106617 s。说明 Guide 的有限碰撞样本与最终 OBB sweep 是独立证据。
- route 0 / solver round 1：连续 F containment FAIL。first_failure：`s=7.757789733 m`、corridor 0、width `9.682634542e-6 m`、depth 17、point residual `-4.691765449e-6 m`、bound residual `+2.059592558e-7 m`；最小宽度阈值为 `1e-5 m`。该记录表示这个 first-failure 中点本身仍在 region 内，但 Lipschitz 区间上界未能闭合。
- 同一 audit 的 aggregate `max_point_residual_m=+2.850885474e-4 m`。由于 `audit_path_in_corridors()` 在每个检查区间记录真实 `path.state_at_s(midpoint)` 的半空间残差，这又表明审核访问过至少一个 F 中点越界；质量报告没有记录 argmax 的 s/corridor，无法定位或核对该越界。故这个 run 不能仅描述为“纯保守宽度限制”，但也不能把 F 点越界称为车体撞墙。

`audit_path_in_corridors()` 的函数体 SHA 与 TASK02 父提交相同；这是 TASK01 既有审查函数中的诊断定位缺口，不是已确认的 TASK02 改动回归。本次不修改阈值或该函数。若后续继续研究该失败，应在独立审核任务中保存 max residual 的位置及对应 corridor，而不是降低 width 阈值。

## 当前无法判断的机制

对用户两条计划，缺少 first failure 全字段、allocation、solver 配点残差、path S、局部 portal 切分及回收记录，无法判断失败来自局部净空、固定 sigma 分配、数值上界过保守、或真实 F 越界。该输入也没有证明三车实际发生 OBB 碰撞。
