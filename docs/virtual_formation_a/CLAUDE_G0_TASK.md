# Claude G0：基线保护核验与安全验证入口

## 0. 可直接执行的任务上下文

你是 Claude Code，实现方；Codex 是独立验收方。本次仅执行 G0，不实现几何、优化、时间、地图或 ROS。用户课题是三辆 Ranger Mini 2.0 固定构型运输；后续新后端统一 A、正向、参考点沿集群前向，保留原 FR06。

正式基线 `567bdecba9676313da3bc9a97b82f57b5b569a2b`，基线分支 `snapshot/formation-v02-fr06-20261115`。未来工作分支 `feature/virtual-rigid-formation-a`，建议隔离 worktree `/home/wq/whut_thesis/expv3_multi_virtual_a`。

**本任务只有用户明确下达、Codex已准备并核验隔离工作区后才可执行。** 若当前仍在 `/home/wq/whut_thesis/expv3_multi_v02` 或快照分支，停止并报告 `WORKSPACE_NOT_PREPARED`。Claude不创建worktree、不操作原仓库、不stash/reset/clean、不提交/推送。

先读本目录 `VIRTUAL_FORMATION_A_DECISIONS.md` 和主方案。若实际代码与本任务冲突，报告真实路径、签名和最小解决方式，不擅改模型。

## 1. 阶段目标与排除范围

交付可复算的基线保护证据、一条不改写历史资料的参考审核入口和安全回归runner。确认真实依赖，不能把未执行算通过。

不新增规划包，不实现G1公式，不运行六次FR06搜索，不启动ROS/Gazebo，不修旧算法。不得因依赖缺失先全局安装/升级软件；先定位已有项目解释器和ROS环境。

## 2. 文件白名单和复用入口

允许新增：

- `tools/virtual_a_baseline.py`：哈希清单、依赖检查、旧参考重新审核和受控回归。
- `evaluation/virtual_formation_a/G0/<run-id>/**`：新证据，唯一目录，不能覆盖既有run。
- 本方案目录的 `G0_INTERFACE_CHECK.md`：真实接口/环境核对；不改研究决策。

不得修改任何现有源码/测试/配置。特别保护：`evaluation/formation_refactor_v02/**`、旧T5工作区、旧地图、车辆YAML、FR01–FR06测试、单车V2和Git引用。

已有真实接口：

- `P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。
- `P/uniform_trajectory.py:568` 的 `UniformTimedPlan`；`from_model_json`、`sample`。
- `P/uniform_quality.py` 的 `UniformAuditRequest`、`audit_uniform_timed_plan(request,plan,projection,grid,config=None)`。
- `P/formation.py` 的 `FormationModel/FormationPose/VehicleOffset`；`P/formation_collision.py` 的 `VehicleShape`；`P/core.py` 的 `Grid`；`P/formation_retiming.py` 的 `FormationTimingLimits`。
- 参考 `evaluation/formation_refactor_v02/FR06/reference/reference_model.json`；地图 `maps/corridor_w2.8/map.pgm`（相对FR06目录）。
- 不直接运行历史 `verify_fr06.py`/`run_experiments.py`，它们会写旧目录；只能读并借用构造方法。

## 3. 新工具签名与输出

在工具中提供：

```python
collect_manifest(repo_root: Path, protected_paths: tuple[str,...]) -> dict
inspect_environment() -> dict
reaudit_fr06_reference(repo_root: Path, width_m: float=2.8) -> dict
run_regression(repo_root: Path, output_dir: Path, level: str) -> list[dict]
main(argv=None) -> int
```

CLI：`python3 -B tools/virtual_a_baseline.py --output DIR --regression baseline|geometry|timing|full`。G0只运行baseline；其余级别建立显式用例清单/环境重定向能力，不能一调用就全目录discover。支持`--verify-manifest FILE`仅核对。

manifest记录精确HEAD/分支、dirty状态、受保护文件SHA-256、当前已知Markdown文档和其它untracked路径。保护范围至少包含旧规划包、旧测试、配置/launch、run_demo、maps及FR06/FR05关键验收证据。新run目录、build/devel/cache不递归纳入保护集合。发现已有未知修改则停止，不试图使工作区“干净”。

新工具若输出目录已存在且非空，应拒绝；不能覆写旧报告。子进程必须保存完整命令、cwd、解释器、环境增量、stdout/stderr和退出码，不能只输出PASS。

## 4. 参考审核的明确原请求

独立构造，而非从被审模型读取目标后自证：

```text
model IDs/offsets = 1:(1,0), 2:(0,1), 3:(0,-1) m
body shapes = .738 × .500 m
start_F=(6,6,0); headings={1:0,2:0,3:0}
goal_F=(10,10,pi/2)
terminal_headings={1:pi/2+atan2(1,2),2:pi/2,3:pi/2}
limits=FormationTimingLimits(); allow_reverse=True（旧参考原合同）
```

从PGM读取`occ=flipud(image<127)`，`Grid(occ,.05,0,0)`；plan_id/formation_id从参考文件取身份即可，不改原文件。调用审核并记录状态、reason、checks、wall_s、duration_s、model摘要。2.8 m期望`FEASIBLE`、时长`29.9552947533 s`。检查数通常13,390，但不把固定检查数或墙钟作为数学合同。

这是旧参考复审，不是新A-only算法验证。历史2.8m成功/3.2m超时和本次参考复审分别标注。

## 5. 环境与历史输出保护

检查当前项目解释器中的numpy/scipy/casadi/PyYAML版本，记录CasADi/IPOPT是否实际可用；import成功不等于插件求解成功，可用极小独立标量问题检查插件，只写新证据，不写算法。缺失时明确阻塞G2，不能假装已有NLP结果。

`rospy`在未source ROS时不可导入，不等于机器没装ROS。记录`/opt/ros/noetic/setup.bash`、`catkin_ws/devel/setup.bash`是否存在；G0不要求启动ROS，也不因此宣称G5验证通过。

源码已确认旧测试有以下可重定向变量，全部须映射到本次run各自子目录：

```text
FR01_RESULTS_PATH
FR02_TIMING_RESULTS_PATH
FR02_TRAJECTORY_RESULTS_PATH
FR02_QUALITY_RESULTS_PATH
FR02_ARTIFACT_DIR
FR03_RESULTS_PATH / FR03_ARTIFACT_DIR
FR04_RESULTS_PATH / FR04_ARTIFACT_DIR
```

先读每个将运行测试的文件；若另有无环境开关的写入，选安全子集或报告，不能修改旧测试来重定向。新工具对FR02-A/B使用不同输出子目录，防止相互覆盖。用subprocess env传值，不用拼接会执行用户字符串的shell命令。

## 6. 开发步骤与阶段内检查

1. G0-A：核验隔离worktree身份及基线祖先，保存before清单；核对原审计JSON、关键证据和源码接口；异常先停。
2. G0-B：实现最小manifest/依赖/参考复审CLI，先执行baseline；确认旧路径零修改。
3. G0-C：列出后续安全回归映射、实际API和依赖阻塞；生成报告。不要开始新数学包。

建议实际命令（未来G0中执行，本轮文档编制未执行）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
git status --short
git branch --show-current
git merge-base --is-ancestor 567bdecba9676313da3bc9a97b82f57b5b569a2b HEAD
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
VA_RUN="evaluation/virtual_formation_a/G0/$(date -u +%Y%m%dT%H%M%SZ)"
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN" --regression baseline
```

时间戳撞名时换唯一run-id，不删除原目录。命令非零要保留失败证据。

## 7. 验收标准和交付物

- 工作区属于已准备的新分支，基线是给定SHA的后代，原两个worktree和历史证据不变。
- before/after受保护文件哈希完全一致；只有白名单新文件增加。
- 参考独立复审`FEASIBLE`；身份和时长匹配；没有搜索或参考注入。
- 依赖、解释器、ROS环境及所有未执行项明确记录；权限导致未执行时标`NOT_RUN_PERMISSION`，不标PASS。
- 工具可从新worktree独立运行，拒绝覆盖run，非零错误返回非零退出码。

交付`COMMANDS.md`、`environment.json`、`protected_before.json`、`protected_after.json`、`reference_reaudit.json`、日志、`source_manifest.json`、`CLAUDE_REPORT.md`及`G0_INTERFACE_CHECK.md`。manifest记录本任务新增工具/文档真实哈希。

## 8. Codex独立验收清单

- 独立检查worktree/分支/基线，不能信Claude截图。
- 从原始快照/旧证据重新计算关键哈希，核查before清单未遗漏关键模块。
- 阅读runner，确认不会调用会写旧证据的历史脚本，确认所有结果变量生效。
- 自己构造原请求重审一次参考，核查目标/车头不是被审模型自报数据。
- 对已存在output、非空目录、缺失模型、错误哈希验证失败路径；不实际破坏旧文件。
- 审核环境阻塞，不因某依赖被skip而宣称已通过所有能力。

**停止条件：输出G0报告后停止。即使全部检查通过，也不得实现G1、创建开发分支或提交Git；等待Codex独立ACCEPTED和用户下达G1。**
