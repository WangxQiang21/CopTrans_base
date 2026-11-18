# Claude D1 单任务：删除已确认失效的 V2 ROS 链接

## 1. 任务性质

这是一个严格白名单的单任务清理。只删除 3 个无目标、无引用的历史 ROS `latest` 符号链接，并保存最小证据。**不是源码重构，不允许探索性删代码，不允许自动扩大范围。**

基线仓库：`/home/wq/whut_thesis/expv3_multi_virtual_a`  
要求分支：`feature/virtual-rigid-formation-a`  
审核基线 SHA：`af7c9fd6246e6c2c1c7b8416d4d094e47f0adc26`

开始前必须确认 HEAD 是该 SHA 或用户明确授权的新后代，并记录已有工作区修改。若存在本任务无法与之明确隔离的修改，停止并报告；不得 reset、checkout 覆盖或清理用户数据。

## 2. 唯一允许删除的对象

```text
evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt1/ros_log/latest
evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt2/ros_log/latest
evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt3/ros_log/latest
```

删除前逐项记录：Git mode、链接文本、目标是否存在、全仓精确路径引用结果。只有仍满足“Git 跟踪的符号链接、目标不存在、全仓无引用”时才可删除；任何一项不满足就停止，不删除其余项目。

## 3. 禁止范围

- 不修改或删除任何 Python/C++ 源码、测试、CMake、launch、消息、配置、地图或脚本。
- 不删除 `planning_display.py`、`test_planning_display.py`、`legacy_route` 或 `formation_continuous_a/` 中任何代码。
- 不修改 `evaluation/formation_refactor_v02/`、`evaluation/virtual_formation_a/`、`qna/`、任何 `reviewed_source/`、PID、验收索引、哈希或 manifest。
- 不修复链接目标，不改成相对链接，不重写 Git 历史，不运行 `git clean`/`git reset`。
- 不把本任务扩展为证据归档、算法优化或格式化。

## 4. 执行步骤

1. 记录 `git branch --show-current`、`git rev-parse HEAD`、`git status --short`。
2. 对 3 个路径分别执行只读检查：`git ls-files -s -- <path>`、`readlink <path>`、`test -e <path>`，并用 `rg -F '<完整相对路径>' .` 检查引用。
3. 条件全部成立后，用 Git 删除这 3 个链接。
4. 检查 diff，确保没有其他变化。
5. 在 `evaluation/cleanup_d1/<UTC时间>/HANDOFF_D1.md` 保存简短交付记录；可同时保存 `commands.log` 和删除前链接信息。不得复制源码或生成大体积证据。
6. 停止，等待 Codex 验收；不提交、不推送，除非用户另行授权。

## 5. 验收命令与量化标准

至少执行并记录等价命令：

```bash
git diff --check
git diff --name-status
git status --short
git ls-files -s -- \
  evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt1/ros_log/latest \
  evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt2/ros_log/latest \
  evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt3/ros_log/latest
```

通过标准：

- 恰好 3 个白名单路径显示为删除；
- `git diff --check` 退出码为 0；
- 没有生产代码、测试、ROS/CMake、配置及既有证据文件变化；
- 3 个链接路径均不再被 Git 索引和工作树保留；
- `HANDOFF_D1.md` 如实记录 BASE SHA、删除清单、检查命令及结果；
- 不声称减少了显著磁盘占用，也不声称完成了源码重构。

失败条件：出现任何白名单外修改、链接仍有效/被引用、基线无法确认，或需要改动现有证据才能删除。发生失败时恢复动作只能在不覆盖用户改动的前提下进行；不确定则停止并报告。

## 6. Codex 独立验收点

Codex 将独立核对 BASE/工作树、三个原始链接在 BASE 中的 mode 与目标、全仓引用、最终 diff 边界和交付记录。由于本任务不改代码，验收不要求重新跑规划器或 ROS；若 diff 出现任何代码变化，则直接 FAIL 并要求撤出超范围变更。
