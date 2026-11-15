# V0.2 工作区准备与基线来源（Codex）

日期：2026-11-12。状态：准备完成，尚未实现 FR01。Claude 只在本工作区实施原语与定向测试，不操作原仓库。

## 执行工作区

- 路径：`/home/wq/whut_thesis/expv3_multi_v02`
- 分支：`refactor/formation-v02`
- HEAD：`6227833011e2d92b711179bfdec979bb0b878b29`（T4 检查点，不声称全部功能稳定）
- 创建后初始 tracked/index/working tree 干净；随后只导入设计输入、未修改的视图辅助工具与本准备证据。
- 没有套用 T5 patch，没有复制 build/devel，没有开始 FR01，没有提交 Git。

## 原工作区保护

原仓库 `/home/wq/whut_thesis/expv3_multi` 保持 `feat/formation-eval` 与原 HEAD。
快照 `/home/wq/whut_thesis/expv3_multi/evaluation/formation_refactor_v02/baseline_20261112T050817765842Z` 保存 673 个 tracked 条目、43 个原有 untracked 条目的清单/哈希、完整 tracked binary diff、关键文件副本。
`T5_original.tar.gz` 保存原 T5 目录60个文件/链接条目（56个普通文件，含忽略日志），已对归档内容逐项校验。
三个 tracked T5 改动仍在原仓库，diff 为782行新增、14行删除。新 worktree 对这三个路径使用 HEAD 的 T4 版本，不导入原未跟踪 T5 测试/运行目录。

本轮只对三份计划文档作用户批准的修订；修订前版本也在快照中。原算法、测试、配置、历史实验和其他原有文件内容未改写。共享 Git 元数据仅新增 worktree/分支登记，无提交、重置、清理或覆盖。

## 受控导入

`evaluation/formation_refactor_v02/preparation/import_manifest.json` 记录六个导入文件的来源与sha256：五份设计/审核文档，以及原样导入的 `tools/configure_formation_view.py`。
该工具是 T4 启动脚本可能需要的历史未跟踪依赖，本轮未修改其代码；它不属于 FR01 新实现。
`READY.json` 是最终准备态核验记录；`protected_source_files.json` 是原 T5 三文件保护指纹，供 Codex 审计使用。Claude无需访问原仓库复核。

## Claude 执行边界

读取 `docs/CLAUDE_TASK_01.md`，核对本地分支/HEAD/输入哈希。只实现 `uniform_motion.py`、`tests/test_uniform_motion.py` 和FR01结果。
准备文件只读；不创建 worktree、不做原仓库备份、不操作原仓库、不提交Git、不进入FR02。
发现准备态不符时停止报告，不能重置或清理。
