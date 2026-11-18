# D1 交付记录：删除 3 个失效 V2 ROS `latest` 链接

- 时间：2026-11-18T10:43:32Z
- 仓库：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- 分支：`feature/virtual-rigid-formation-a`
- BASE SHA：`af7c9fd6246e6c2c1c7b8416d4d094e47f0adc26`
- 基线工作区：已有未跟踪目录 `plan/cleanup_d1/`（任务书与独立审核文档），本任务未修改它。

## 删除前检查

三个路径均为 Git mode `120000` 的已跟踪符号链接，`readlink` 均为绝对路径，`test -e` 均返回 1（目标不存在）。原始目标文本见 [links_before.txt](links_before.txt)。

逐项执行原始 `rg -F '<完整相对路径>' .` 时，命中仅为未跟踪的 `plan/cleanup_d1/CLAUDE_D1_TASK.md` 与 `plan/cleanup_d1/D1_INDEPENDENT_REVIEW.md` 中列明本白名单的文字，没有运行输入或源码引用。随后两项复核均通过：

- 排除任务/交付目录 `plan/cleanup_d1/**`、`evaluation/cleanup_d1/**` 后，三个 `rg -F` 均退出 1（无引用）。
- 对 Git 已跟踪内容执行 `git grep -F`，三个路径均退出 1（无引用）。

## 删除清单

仅执行一次 `git rm -- <三个白名单路径>`：

1. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt1/ros_log/latest`
2. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt2/ros_log/latest`
3. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt3/ros_log/latest`

未修改源码、测试、ROS/CMake、launch、消息、配置、地图、脚本或既有证据文件。

## 验收结果

| 命令 | 结果 |
|---|---|
| `git diff --check` | 退出 0 |
| `git diff --cached --check` | 退出 0 |
| `git diff --name-status` | 无未暂存 diff，退出 0 |
| `git diff --cached --name-status` | 恰好上述 3 个路径为 `D` |
| `git diff --cached --summary` | 恰好 3 条 `delete mode 120000` |
| `git ls-files -s -- <三个路径>` | 无输出，退出 0；索引不再保留 |
| `test -e` / `test -L` | 三个工作树路径均不存在 |

`git status --short` 的已跟踪变化仅为这 3 个暂存删除；另有任务前已存在的 `plan/cleanup_d1/` 和本交付目录两个未跟踪目录。

按任务要求未运行规划器、ROS 或测试；本任务无代码变化。未提交、未推送。这里只删除 3 个失效目录项，不声称显著减少磁盘占用，也不声称完成源码重构。停止并等待 Codex 验收。
