# 本地实验日志与复现

`scripts/interactive.sh` 和 `scripts/test.sh` 都保存紧凑的本地实验日志。编号形如 `20261125_000000_aB1cDe`，包含本机日期、时间和短后缀。RViz 的「规划与执行」面板和终端都显示编号。遇到问题时，把编号和观察到的现象发给开发人员即可定位这一轮。

## 保存内容

| 文件 | 用途 |
| --- | --- |
| `experiment.json`、`result.yaml` | 编号、起止时间、起终点坐标和航向、规划或执行结果、失败代码与说明、规划用时及跟踪指标 |
| `trajectories/planner.cdr.gz` | 无损保存原始规划消息。三车消息内含共同时间轴、三车和货物轨迹、权威模型及摘要。规划失败时 `plan_artifact` 为空；超大信封占用过多余量时可保留 `trajectories/model.json.gz` 的权威解析路径 |
| `resolved_config.yaml`、`planner_parameters.yaml`、`controller_parameters.yaml`、`scene/body_twist_parameters.yaml`、`environment.txt` | 复跑配置，包括规划器和跟踪器、限值、Git 提交和工作区是否有修改 |
| `scene/map.pgm.gz`、`scene/map.yaml`、`scene/scene_meta.yaml` | 原始占据栅格及场景快照。归档同时绑定地图和场景元数据的 SHA-256 |
| `operator_events.jsonl.gz`、`planning_progress.jsonl.gz`、`planner.log.gz`、`timing_trace.jsonl` | 选点和拒绝原因、候选统计、认证阶段、准入及共同起步事件 |
| `metrics/motion.jsonl.gz`、`controller.log.gz` | 常态约 2 Hz 的位置、航向、速度、Twist、模式、控制误差和间距趋势。控制器模式变化、预检和停车等事件额外记录。仿真验收仍使用原有测量频率 |
| `metrics/fault_tail.jsonl.gz` | 出现控制或执行故障时保留故障前最多 8 秒、最多 1,000 条未经降采样的控制状态，便于核对异常瞬间的指令和模式 |

原始实验目录实测有一轮约 8.2 MB，逐周期控制日志约 4.9 MB。用这份历史数据估算，新策略的无损轨迹、控制趋势和位姿趋势三项压缩后合计约 227 KB。新交互的成功短弯和地图外拒绝用例，所存文件分别约 68 KB 和 21 KB。常态趋势与状态变化帮助区分规划失败、控制准入失败、意外换模和跟踪误差，故障时间窗补回高频上下文。普通试验默认关闭全量 bag。一次性诊断可显式设置 `--record on`，能归档的 bag 同样计入项目配额。

## 容量规则

归档保存在仓库的 `reports/runs/`，由 `.gitignore` 排除。仿真生成的重复世界文件和完整运行时临时文件在该目录之外生成，归档完成后清理。常规归档的预算为每轮 4 MB，优先保留起终点、结果、场景配置和路径，空间不足时省略诊断文件并在 `omitted_by_budget` 列明。现有实验约占 0.55 GB，再增加 1,000 轮常规预算约到 4.55 GB。已有实验数据也计入 **5 GB** 的本地总配额。接近上限时脚本暂停创建新实验，已有日志保留。重型手工录包和超大权威轨迹可能提前用完总配额。

## 查日志和复跑

```bash
cd /home/wq/Whut/CopTrans_base
id=20261125_000000_aB1cDe  # 换成面板显示的编号
cat "reports/runs/$id/experiment.json"
gzip -cd "reports/runs/$id/planner.log.gz"
./scripts/interactive.sh --replay "$id" --no-build
```

复跑创建新编号，并在 `replay_of` 字段登记原编号。脚本自动载入当时的选点与参数，核对地图和场景摘要，再走正常规划和三车准入链。它用当前安装的算法重新求解，因此可以验证修改是否修复问题。检测到不同 Git 提交时会提示。原日志继续可查。提交规划请求前中断的实验可检查状态和选点事件。
