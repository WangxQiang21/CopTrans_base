# 4WIS 新规划架构资料库

本目录集中保存下载到的论文、论文/项目架构拆解，以及针对现有规划器提出的新架构。原始工作区 `exp/exp_plan/documant/` 保持不变。

## 从这里开始

1. [新架构提案](NEW_ARCHITECTURE.md)：问题重述、模块分层、关键算法决策、实施顺序与验收条件。
2. [Chang 2026 复现计划](CHANG_2026_REPRODUCTION_PLAN.md)：论文方法拆解与总体复现方案。
3. [逐阶段执行手册](CHANG_2026_EXECUTION_RUNBOOK.md)：每阶段任务顺序、实验矩阵、产物和 go/no-go。
4. [论文 × 当前代码模型审计](CHANG_MODEL_AUDIT.md)：当前实现与论文架构的差异、参数冲突和 Phase 0 阻塞项。
5. [论文与项目架构分析](analysis/PAPER_PROJECT_REVIEW.md)：逐篇论文和逐个项目的架构、适用点与限制。
6. [资料索引](PAPERS_AND_PROJECTS.md)：本地 PDF、论文 DOI/原文页、代码仓库入口和下载状态。
7. [当前空间模型与 S09 仿真结果](experiments/results/summary/phase1_spatial_pilot.md)：中文结果说明、逐目标路径图和当前模型限制。路径仍是离散搜索原型，连续几何优化待实现。
8. [阶段验收说明](experiments/results/summary/阶段验收说明_当前空间规划原型.md)：解释这一阶段的目标、已完成与未完成证据，并给出是否进入下一阶段的判断。
9. [固定模式连续优化结果](experiments/results/summary/固定模式连续优化_阶段结果.md)：说明粗路径、安全走廊、连续优化和独立碰撞裁判之间的关系，并给出 S09 优化前后对比。

## 目录

- `papers/`：从 `exp/exp_plan/documant/` 复制来的本地 PDF（原文件名保留）。
- `analysis/`：文献与代码架构分析。
- `experiments/`：独立空间规划原型、配置快照、运行数据和阶段报告。

## 当前方案基线

按当前源码审计：原规划器默认前端是 2D 宏观通道加 Spin/Ackermann 序列；后端是自由 body-twist NLP，但默认锁住优化几何后由阈值回判模式；因此它既未实现 Chang 的离散模式/几何联合优化，也未实现模式切换 ROI 瞬态。另有一个独立理想空间运动/碰撞及 S09 搜索原型，完成了小规模 smoke/pilot 运行，但阶段验证未过门：未做系统性运动学差分、独立 OBB swept 对照，也未实现 STC。当前建议留在 Phase 1，先补验证，不进入连续优化阶段。研究范围经用户确认限定为几何规划并假设完美控制跟踪；轮级执行和闭环控制留到下一阶段。车型参数和源模型仍有参考点、模式映射、margin 链、求解器等待核项。详见 [`CHANG_MODEL_AUDIT.md`](CHANG_MODEL_AUDIT.md)、[`planning_scope.md`](experiments/decisions/planning_scope.md) 和[阶段验收说明](experiments/results/summary/阶段验收说明_当前空间规划原型.md)。

> 论文中的结果是作者在各自平台和假设下报告的结果，不作跨论文直接排名。本方案是待实现/待对照的工程设计，不声称已验证。
