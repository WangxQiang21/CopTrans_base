# 规则与强化学习前端：开发、训练与检查

当前主线为 **规则候选生成＋RL 选择模式与路径分支**；独立离线前端已可运行，`run_demo.sh` 仍运行父版本 v2。
阶段验收结果及图片见 [`evaluation/rl/foundation/REPORT.md`](../evaluation/rl/foundation/REPORT.md)。
首轮 RTX 4060 训练曲线、实际成功/失败路径和原始结果索引见 [GPU 训练评估](../evaluation/rl/training/REPORT.md)。

新前端的规则版为短程验证 20/20、工程地图 3/11；512 步 GPU 接入小试尚未优于规则版。
见 [新前端逐图评估及图片](../evaluation/rl/rule_frontend/REPORT.md)。旧 80 动作模型只有 2/20，保留为历史实验。

## 当前主线：独立规则与学习前端

本入口直接接收地图和起终点，不调用旧 C++ 完整搜索，也不调用 `ModePathMaster.solve`。
复用现有 C++ A 模式局部连接公式，并生成 O、Spin、短圆弧、引导点和径向候选。
需要 `g++`；首次使用会将局部连接库编译到 `/tmp/coptrans-rl-native/<源码哈希>/`，不修改 ROS 工作空间。

策略一次选择一个“路径前缀＋局部连接”，同时决定探索哪条分支、哪种模式和哪段几何。
未选择的分支保留在有容量限制的候选集合中；规则版用固定优先级，学习版在其上学习评分修正。
学习权重初始为零时，确定性选择与规则版一致。所有完整路线经过共同完成器，按审核通过后的实际执行时间选择。

安装下文 CUDA 学习环境后，可单独运行：

```bash
cd /home/wq/whut_thesis/expv3
DATA=/home/wq/whut_thesis/expv3_experiment_archive/rl/datasets/curriculum-v1-20261026
RUN=/home/wq/whut_thesis/expv3_experiment_archive/rl/training

# 规则版：固定的前 20 个验证目标
.venv-rl/bin/python -m coptrans_rl.rule_learning evaluate \
  --dataset "$DATA" --output /tmp/rule-val-new-01 \
  --split val --episodes 20 --budget 5 --device cpu

# 规则版：11 张工程地图，默认比例及预设起终点
.venv-rl/bin/python -m coptrans_rl.rule_learning evaluate \
  --dataset "$DATA" --output /tmp/rule-maps-new-01 \
  --maps --budget 10 --decisions 512 --device cpu

# GPU 接入小试；512 步仅用于验证学习流程
.venv-rl/bin/python -m coptrans_rl.rule_learning train \
  --dataset "$DATA" --output "$RUN/rule-my-run-01" \
  --device cuda --steps 512 --rollout-steps 128 --budget 5 --seed 7

# 评估实际训练后的权重；best.pt 可能仍是未更新的规则基线
.venv-rl/bin/python -m coptrans_rl.rule_learning evaluate \
  --dataset "$DATA" --model "$RUN/rule-my-run-01/latest.pt" \
  --output /tmp/rule-rl-val-new-01 --episodes 20 --budget 5 --device cuda

# 生成实际路径及学习曲线
.venv-rl/bin/python -m coptrans_rl.rule_figures \
  --dataset "$DATA" --evaluation /tmp/rule-rl-val-new-01/evaluation.json \
  --run "$RUN/rule-my-run-01" --output /tmp/rule-rl-figures-new-01
```

输出目录必须尚不存在，且在仓库外。`evaluation.json` 保存逐任务定时轨迹、审核、搜索事件、失败原因及最后尝试。
`best.pt`、`latest.pt` 是新候选选择网络，不能与旧 80 动作模型的 zip 权重互换；源码/车辆身份不符会拒绝加载。
当前 PPO 小试采用单个 CPU 几何环境采样、GPU 网络更新；旧 80 动作训练入口的 4 进程设置不适用于这个新入口。
本轮证明了接口和 CUDA 更新流程，尚未证明强化学习改善路径质量。正式训练前需补齐长距离任务和候选覆盖。

## 已实现

- 固定编号的 80 个动作：A 30 个、O 42 个、Spin 8 个。
- 同一车辆配置、地图尺度、方向及不揉库规则；掩码包含完整车体的中间姿态扫掠。
- Gymnasium 环境：局部图、全局图、状态与历史；成功奖励按最终执行时间校正。
- 搜索之外的候选完成器：连续化、终点拟合、模式边界/后缀修复、连续计时、独立审核、显示插值扫掠。
- 11 图输入清单、三版原生计时对比、真实 ROS 回放检查、随机/贪心/确定动作环境检查。

当前完成器不调用 `ModePathMaster.solve` 或完整传统搜索。它复用了 v2 的基础函数；
现有 v2 在线管线仍使用自己的调度代码，尚未改为调用这个新入口。
当前几何修复为有界终点、模式边界及后缀拟合，保持模式/方向序列；完整固定结构 NLP 优化和解析末端连接器仍属后续阶段。

## 独立 Python 环境

学习环境使用本机 Python 3.12，ROS 保持系统 Python 3.8。不要在 ROS 解释器中安装学习依赖。

```bash
cd /home/wq/whut_thesis/expv3
UV_CACHE_DIR=/tmp/coptrans-uv-cache uv venv --python /home/wq/.local/bin/python3.12 .venv-rl
UV_CACHE_DIR=/tmp/coptrans-uv-cache uv pip install --python .venv-rl/bin/python -r rl/requirements-foundation.lock
UV_CACHE_DIR=/tmp/coptrans-uv-cache uv pip install --python .venv-rl/bin/python --no-deps -e ./rl
```

锁文件对应 Linux x86_64 / Python 3.12 的基础环境，不包括 PyTorch、PPO 和训练依赖。
`.venv-rl`、安装产物、训练数据及权重均不提交到 Git。

## 学习依赖安装：默认使用 GPU

本机已在宿主环境确认 RTX 4060 / 8 GB 显存，驱动 570.181 / CUDA 12.8。
训练命令在能运行 `nvidia-smi` 的普通宿主终端执行。受限工具环境可能看不到 GPU，不能把这一现象当成驱动故障。

先安装 CUDA 学习环境并验证实际 GPU 运算：

```bash
cd /home/wq/whut_thesis/expv3
git switch rl-mode-path
bash rl/setup_training.sh
```

安装使用官方 PyTorch 2.8.0 CUDA 12.8 包、Stable-Baselines3 / sb3-contrib 2.9.0。
完整运行依赖已锁在 `requirements-training-cu128.lock`，安装脚本同时验证真实 CUDA 运算。
实现遵循 [MaskablePPO 官方接口](https://sb3-contrib.readthedocs.io/en/master/modules/ppo_mask.html)：环境内部提供掩码，训练和评估均传递掩码。

## 历史路线：80 动作自由生成实验

下面命令用于保留的旧训练路线，不是当前候选选择前端。该路线尚未通过规划效果验收。

### 1. 生成入门课程数据

已生成的数据位于以下目录，可以直接使用；完整数据和权重放仓库外。

```bash
DATA=/home/wq/whut_thesis/expv3_experiment_archive/rl/datasets/curriculum-v1-20261026
RUN=/home/wq/whut_thesis/expv3_experiment_archive/rl/training
BC_RUN="$RUN/bc-my-run-01"
PPO_RUN="$RUN/ppo-my-run-01"
```

该数据集包含训练 1,000、验证 200、测试 500、另留地图类型 100 个任务。布局哈希跨集合隔离。
教师是通过同一 80 动作库构造并经完整审核的可达样本；尚未采集三个传统算法的教师，不能把这批短程课程当成论文正式盲测。

需要重新生成时，使用一个尚不存在的目录：

```bash
.venv-rl/bin/python -m coptrans_rl.dataset --output /tmp/rl-data-new
```

### 2. 行为克隆预训练

```bash
.venv-rl/bin/python -m coptrans_rl.train bc \
  --dataset "$DATA" --output "$BC_RUN" \
  --device cuda --epochs 30 --seed 7
```

策略学习教师在每一步的动作选择；验证集只用于评估和选模型。
示例中的 `BC_RUN`、`PPO_RUN` 必须是新目录；重复运行时修改名称。已完成的 `bc-seed7*`、`ppo-seed7-goal-v2-pilot` 为冻结实验，保留作对照。
当前目标位移以车长归一化；旧 `bc-seed7` 使用旧观测，不能加载到新观测代码中，入口会拒绝不匹配的权重。
输出 `bc_history.json`、`best_validation.json`、`best_model.zip`、`latest_model.zip` 和带依赖/配置/数据/权重哈希的 `manifest.json`。
`best_model` 按验证到达率优先、成功轨迹执行时间其次选择；仅验证 NLL 改善不能替代实际到达评估。

### 3. MaskablePPO 微调

```bash
.venv-rl/bin/python -m coptrans_rl.train ppo \
  --dataset "$DATA" --output "$PPO_RUN" \
  --initial-model "$BC_RUN/best_model.zip" \
  --device cuda --envs 4 --timesteps 200000 --seed 7
```

GPU 训练 CNN、策略和价值网络；4 个 CPU 进程进行几何环境采样。每个环境 512 步，合计 2,048 transitions 后更新。
请求步数按完整 rollout 向上取整，实际步数保存在 `training_summary.json`。训练日志 `progress.csv` 包含损失、KL、近期到达率和实际吞吐。
每 50,000 transitions 保存 checkpoint，并在固定验证任务上选最优模型。

中断后可从已有检查点继续；`--timesteps` 表示本次追加步数：

```bash
.venv-rl/bin/python -m coptrans_rl.train ppo \
  --dataset "$DATA" --output "$RUN/ppo-my-run-01-resume1" \
  --initial-model "$PPO_RUN/checkpoint_50000.zip" \
  --device cuda --envs 4 --timesteps 150000 --seed 7
```

续训保留权重与已有步数，以指定 seed 重新建立采样状态；不是逐位恢复此前的随机流。
输出目录必须尚不存在，避免覆盖旧实验。训练入口默认 CUDA；显式 CUDA 不可用时会报错，CPU 合同测试可显式指定 `--device cpu`。

### 4. 单独评估模型并生成图片

```bash
.venv-rl/bin/python -m coptrans_rl.evaluate \
  --dataset "$DATA" --model "$PPO_RUN/best_model.zip" \
  --split test --episodes 500 --device cuda \
  --output "${PPO_RUN}-test"

.venv-rl/bin/python -m coptrans_rl.training_figures \
  --dataset "$DATA" --run "$PPO_RUN" \
  --evaluation "${PPO_RUN}-test/evaluation.json" \
  --output "${PPO_RUN}-figures"
```

图片包含课程地图/教师预览、实际学习曲线及模型成功和失败路径。`--split ood` 单列未参加训练的 islands 类型。
训练与选模型使用验证集；测试集留到配置冻结后再评估。当前课程数据上的成绩与 11 张工程地图、正式盲测分别报告。

## 一次运行基础检查

`--output` 必须是尚不存在的目录；每次测试创建独立目录，不覆盖旧证据。

```bash
# 内核测试、环境合同测试、11 图输入清单、原生计时图、环境轨迹图、跨解释器检查
bash rl/run_checks.sh --output /tmp/coptrans-rl-check-01

# 再增加真实 ROS 回放验证；先保证 catkin_ws 已成功构建
bash rl/run_checks.sh --output /tmp/coptrans-rl-check-02 --ros
```

ROS 检查会启动独立 master 和受控仿真时钟，使用现有 `playback.py` 发布车体 Marker。
它检查真实消息序列化和回放插值；完整规划器、GUI 帧率和真实车辆响应不在这项检查范围内。
固定驻留按输入轨迹时间戳以 1e-6 秒容差验证；Marker 约 40 ms 采样会低估静止区间，另报最多两个采样周期的不确定度。

也可单独运行：

```bash
.venv-rl/bin/python -m unittest discover -s rl/tests -p 'test_*.py'
.venv-rl/bin/python -m coptrans_rl.foundation --output /tmp/coptrans-rl-foundation-01
```

主要产物：

| 文件 | 用途 |
|---|---|
| `foundation/manifest.json` | 车辆、配置、源文件、地图、起终点和动作库身份 |
| `foundation/native_timing.json` / `.png` | 三版原生驻留、Spin、低速平移、内部零速点和时间曲线 |
| `foundation/mask_audit.json` | 优化掩码与标量参照的逐图一致性和耗时 |
| `foundation/environment_smoke.json` / `.png` | 四种基础任务的动作、审核、奖励核对和轨迹 |
| `kernel-parity.json` | ROS 与学习解释器执行公共完成器的数值差异 |
| `ros-playback/playback_audit.json` | 实际 Marker 序列、插值误差和驻留观测 |

图中的 `known_valid` 是确定动作重放，`distance_greedy` 和 `random` 是无学习策略。
这些结果用于检验环境及审核流程，不能据此宣称强化学习规划效果。

## 模块与边界

| 模块 | 职责 |
|---|---|
| `local_connections.py` / `rl/native/local_connections.cpp` | 调用现有 C++ 短程 A 连接公式，无完整路径搜索 |
| `rule_candidates.py` | 多方向引导点、A/O/Spin 局部连接及整车扫掠 |
| `rule_frontend.py` | 分支保留、有限探索、共同完成器和实际执行时间选择 |
| `rule_learning.py` | 共享候选评分网络、规则残差 PPO、离线评估 |
| `rule_figures.py` | 成功定时路径、失败尝试和学习曲线 |
| `kernel_adapter.py` | 读取公共车辆/计时配置，导入不依赖 ROS 的内核 |
| `configuration.py` | 版本化环境设置与配置身份 |
| `benchmark.py` | 按 ROS 栅格语义读取 11 图并按中心缩放 |
| `action_library.py` | 动作编号、模式、长度/角度/曲率 |
| `action_mask.py` | 参照扫掠和保守快速筛选后的批量 SAT |
| `observation.py` | 米制局部裁剪、全局地图和历史；原地图验碰保持原分辨率 |
| `environment.py` / `reward.py` | 环境转移、任务终止、掩码接口和奖励 |
| `candidate_finalizer.py`（公共内核） | 同模式连续计时及最终发布资格检查 |

起点即终点的精确零运动任务由完成器单独认证，耗时为 0；训练环境排除已在终点容差内的任务。
动作步数耗尽是任务失败，返回 `terminated=True`；非法编号、全无效掩码和审核失败都有明确原因。
环境中的一步是几何决策，同模式同方向动作会连续运行；不会因为策略做了多次决策而增加停车。

## 下一阶段

先补齐传统教师、基础动作课程和策略错误分解，提高多步验证到达率；再完成多种子训练。
之后接入独立 ROS RL 前端，评估 11 图和盲测任务。
完整阶段门槛见 [`plan/rl/02_执行方案与验收标准.md`](../plan/rl/02_执行方案与验收标准.md)。
