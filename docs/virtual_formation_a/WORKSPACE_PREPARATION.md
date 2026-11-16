# Codex 工作区准备交接：可由 Claude 执行 G0

日期：2026-11-15。**状态：WORKSPACE_READY；G0 尚未由 Claude 实施或经 Codex 验收。**

## 1. 分工与当前停止点

1. **Codex：** 保存原工作区与基线、创建并核验隔离 worktree、复制必要设计文档。本项已完成。
2. **Claude Code：** 仅在准备好的新 worktree 中，按 `CLAUDE_G0_TASK.md` 实现 G0 工具、运行规定核验并交付真实结果。本项待执行。
3. **Codex：** 收到 Claude 的 G0 交付后，独立读差异、核实证据、运行必要复核，再决定是否签发 G0 的 `CODEX_ACCEPTANCE.json`。本项尚未进行。
4. **用户：** G0 通过后再明确授权 G1。任何一方不得自动进入 G1–G5。

**G0 是首个阶段，不要求先存在 G0 的 ACCEPTED；它的前置条件是本工作区准备完成。** 历史 FR01–FR06 验收不能代替新 G0 验收，也不是新算法各阶段的完成证明。

## 2. 已准备的工作区

| 项目 | 实际值 |
|---|---|
| 工作目录 | `/home/wq/whut_thesis/expv3_multi_virtual_a` |
| 分支 | `feature/virtual-rigid-formation-a` |
| HEAD | `567bdecba9676313da3bc9a97b82f57b5b569a2b` |
| 基线关系 | 当前 HEAD 恰好等于指定 FR06 基线，无新增提交 |
| 任务书 | `docs/virtual_formation_a/CLAUDE_G0_TASK.md` |
| 活动工作区中的 G0 工具/结果 | 尚无，交由 Claude 实施 |
| G1 新规划包 | 尚无 |

Claude 必须先 `cd` 到上表新目录；不得在原 `/home/wq/whut_thesis/expv3_multi_v02` 执行实现，不自行创建 worktree/切分支，不操作原仓库，不提交或推送。

## 3. 基线与资料保护

- 原 T5 工作区保留在 `snapshot/formation-t5-20261115`，HEAD `4043227f71009de7628fe45e9ac0015865df47e2`。
- 原 V0.2 工作区保留在 `snapshot/formation-v02-fr06-20261115`，HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`。
- 永久本地备份：`/home/wq/whut_thesis/_virtual_a_baseline_protection/20261115T134438Z/`。
- 备份包含两个快照分支的完整历史 Git bundle、原工作区状态/文件哈希/差异、九份未提交 Markdown 及两份忽略的历史研究 JSON 归档。构建目录和缓存仅记录清单，没有复制、覆盖或清理。
- 十份必要设计资料已按原字节复制并保存哈希：八份方案/任务书、原技术审核报告、下载目录原稿。源文件保持不变。
- 原稿副本为 `docs/virtual_formation_a/SOURCE_REVIEW_PLAN_FR06_BASE.md`；审核报告为 `docs/VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md`。

## 4. 已纠正的越界工作

Codex曾误将“准备工作区”扩大为代替Claude执行G0，提前生成工具和核验记录。用户纠正后，这些新产物已全部从活动worktree移出，按SHA-256核对后保留在：

`/home/wq/whut_thesis/_virtual_a_baseline_protection/20261115T134438Z/codex_preemptive_g0/`

共31个文件；没有删除历史资料，没有改动原算法、测试或配置。这些产物**不算Claude的G0交付，不构成G0验收，不要求Claude沿用**。未签发新G0的`CODEX_ACCEPTANCE.json`。

活动worktree只保留基线和设计文档，包括本准备交接说明。后续恢复“Claude实现、Codex独立验收”的分工。

## 5. 给 Claude 的执行指令

> 请在 `/home/wq/whut_thesis/expv3_multi_virtual_a` 执行 `docs/virtual_formation_a/CLAUDE_G0_TASK.md`。Codex已经完成隔离worktree和基线保护，当前分支为 `feature/virtual-rigid-formation-a`，HEAD为指定基线SHA。先核验本说明和实际Git状态，再仅实现、测试和交付G0；不要因尚无新G0的ACCEPTED而等待循环前置条件。不得操作原worktree、提交Git或进入G1。完成后停止，交由Codex独立验收。
