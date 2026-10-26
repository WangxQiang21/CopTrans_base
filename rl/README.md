# 强化学习版本开发与基础检查

当前实现到 **P0/P1 基础模块**，没有训练好的策略；`run_demo.sh` 仍运行父版本 v2。
阶段验收结果及图片见 [`evaluation/rl/foundation/REPORT.md`](../evaluation/rl/foundation/REPORT.md)。

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

P2 生成布局隔离的数据集及同库教师；P3 完成 BC、MaskablePPO 训练和吞吐实测；
之后才接入独立 ROS RL 前端，评估 11 图和盲测任务。
完整阶段门槛见 [`plan/rl/02_执行方案与验收标准.md`](../plan/rl/02_执行方案与验收标准.md)。
