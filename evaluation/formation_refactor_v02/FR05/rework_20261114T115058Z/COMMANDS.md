# FR05-R2 发布前空摘要守卫：真实命令与退出码

工作区 `/home/wq/whut_thesis/expv3_multi_v02`（worktree，未提交 Git）。
`R` 为本目录：

```bash
R=/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR05/rework_20261114T115058Z
cd /home/wq/whut_thesis/expv3_multi_v02
```

`env.sh` 只用于 source `/opt/ros/noetic` 与本 worktree
`catkin_ws/devel`（`OPENBLAS_NUM_THREADS=1`、专用 `ROS_HOME`）。
本轮**没有启动 roscore 或任何 ROS 节点**：两项验收都直接驱动真实
`UniformPlannerNode.on_playback_status()`。

## 1. 反馈身份单测（由最终源码新运行）

```bash
source "$R/env.sh"
python3 -m unittest -v tests.test_uniform_feedback_identity \
  > "$R/logs/units_feedback_identity.log" 2>&1
echo EXIT=$?
```

实测 `EXIT=0`，10 项全部 `ok`：

```
test_matching_identity_terminal_releases_busy ... ok
test_missing_identity_terminal_does_not_release_busy ... ok
test_prepublication_explicit_null_digest_terminal_is_ignored ... ok
test_prepublication_missing_digest_start_does_not_confirm ... ok
test_prepublication_missing_digest_terminal_is_ignored ... ok
test_published_planning_terminal_releases_busy ... ok
test_published_zero_motion_terminal_releases_busy ... ok
test_wrong_digest_hold_start_does_not_confirm_playback ... ok
test_wrong_digest_terminal_does_not_release_busy ... ok
test_wrong_formation_or_backend_terminal_is_ignored ... ok

Ran 10 tests in 0.004s
OK
```

## 2. Codex 反馈阶段边界探针（原输入、原脚本，复制到本目录运行）

为避免覆盖 Codex 验收目录中的原始输出，先把脚本逐字节复制到本目录
（`cmp` 确认相同）再运行；输出因此写在本目录。

```bash
source "$R/env.sh"
python3 "$R/feedback_phase_boundary_probe.py" \
  > "$R/logs/feedback_phase_boundary_probe.log" 2>&1
echo EXIT=$?
```

实测 `EXIT=0`；脚本自身打印的 JSON 与
`feedback_phase_boundary_probe.json` 内容一致。

## 3. 探针预期断言（新增，只读取上述 JSON）

```bash
python3 -I "$R/check_probe_expectations.py" \
  > "$R/logs/probe_expectations_check.log" 2>&1
echo EXIT=$?
```

实测 `EXIT=0`：

```
original_wrong_digest                final=PLAYING  seen=False events=0 ok
published_matching_digest            final=HOLD_END seen=True  events=2 ok
prepublication_missing_digest        final=PLANNING seen=False events=0 ok
prepublication_null_digest           final=PLANNING seen=False events=0 ok
prepublication_missing_start_digest  final=PLANNING seen=False events=0 ok
PROBE_ASSERTIONS_OK
```

## 4. 清单生成与校验

清单沿用上一轮 174 个源码/测试/配置路径，逐项重新哈希，并加入本目录
复制的探针脚本，共 175 项：

```bash
old=evaluation/formation_refactor_v02/FR05/rework_20261114T095659Z/SOURCE_MANIFEST.sha256
cut -d' ' -f3- "$old" > "$R/logs/.paths"
: > "$R/SOURCE_MANIFEST.sha256"
while IFS= read -r p; do sha256sum "$p" >> "$R/SOURCE_MANIFEST.sha256"; done \
  < "$R/logs/.paths"
rm "$R/logs/.paths"
sha256sum "$R/feedback_phase_boundary_probe.py" >> "$R/SOURCE_MANIFEST.sha256"

sha256sum -c "$R/SOURCE_MANIFEST.sha256" > "$R/logs/manifest_check.log" 2>&1
echo EXIT=$?
```

实测 `EXIT=0`，175/175 全部成功（本机 `sha256sum` 输出“成功”，日志中无
“失败/警告/错误”）。与上一轮清单逐行比较，只有两处哈希变化（本轮允许
修改的文件）加一条新增探针路径。

## 未运行项（按 ACCEPTANCE §3 明确不要求）

- `tests/ros_uniform_playback_smoke.py` 的 main / near_goal / failure / tamper
  四套 ROS 场景；
- RViz 鼠标 GUI 流程；
- 上一轮 182 项全套回归。

守卫只位于 `on_playback_status()` 的三个确认状态分支之前，未触及搜索、
时间规划、回放算法、launch 默认值或任何其他路径；这些未改路径的证据按
下文 REPORT 的“继承”表引用上一轮真实记录，不冒称本轮重跑。