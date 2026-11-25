# 三车局部重规划和同步切换原型

原型只在离线调用。复用已通过独立审计的 `uniform_v02` 联合模型和真实规划器，使用来自同一单调钟的绝对纳秒。尚无 ROS 2 控制接入和实车验收。

1. 捕获运行中三车的权威计划、模型与投影摘要、请求指纹、地图版本和运行起点。以原计划未来已有的静止准备相位作为共同交接窗口。起点携带上一条边的轮端准备状态，终点是下一共同停止点，申请的路径长度和参考时长均受限。
2. 单线程 worker 在独立墙钟时间和扩展预算内运行局部请求。观察器可立即返回，规划在其他线程继续。worker 验证地图和完整请求指纹、候选模型与独立审计的碰撞和连续动力学认证、两端位置与车身航向、初始相位和局部窗口长度。无可执行计划、超时和不匹配时保持仍有效的原计划。超时任务退出后才接受下一次申请。地图变动或者原计划失效时请求整组停车。
3. 可用候选生成带地图版本、新 plan ID、新模型摘要和唯一生效时刻的准备记录。三辆车均需在现有静止窗口内返回新鲜且完全匹配的确认。启动仍受所有确认和同一未来时刻约束。错误身份、过期、重复时间戳、未知车号和只有部分车辆确认均无法放行。计划结束时停车，后续续接由另一次联合准入决定。

[可复用接口](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/local_replan_protocol.py) · [九项回归](../../ros2_ws/src/fwis_backends/test/test_local_replan_protocol.py)

## 复现

从仓库根目录执行。启用测试所用 Python 科学计算包和 pytest。

```bash
export PYTHONPATH="$PWD/ros2_ws/src/fwis_backends:$PWD/ros2_ws/src/ranger_mini_gz"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m pytest -q ros2_ws/src/fwis_backends/test/test_local_replan_protocol.py
python3 reports/architecture_optimization/phase7_offline_replay.py --output /tmp/local-replan-10hz.json
```

复测以室内短弯的真实独立认证计划为输入。单次计划请求在后台生成实算局部候选，离线时钟按 10 Hz 轮询 120 次。回放先故意发送错误生效时刻，再模拟三车在同一停止区间发回匹配确认。观察到唯一共同切换和新模型进入执行状态。第 110 个采样修改地图版本，之后所有采样都指示整组停车。

本次轮询处理耗时 P99 为 0.052 ms，最大为 0.075 ms。采样唤醒延迟 P99 为 0.188 ms，最大为 5.225 ms。九项回归测试通过。

[P50、P95、P99 和全部轮询样本](phase7_offline_10hz.json)

上线前需验证控制器的实际停稳、轮角和模式与续接边一致，接入地图版本的可信来源和同一 ROS 时钟域，落实三车的原子协调发布和共同停车，并在真实执行环境下测量调度、通信和切换延迟。原型的墙钟测量覆盖离线调度及协议处理。
