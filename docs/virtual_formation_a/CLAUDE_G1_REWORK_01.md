# Claude G1 返工 01：修正 Simpson 积分余项上界

## 背景与范围

G1 几何测试 20 项、profile 测试 16 项通过，双移线数值与基准一致。Codex 审核发现 Simpson 积分余项预算与实际面板宽度不一致，G1 暂不验收。

只返工 G1 的积分误差预算及其必要测试和证据。不得进入 G2，不得修改旧 FR06 算法、测试、配置或历史证据，不提交 Git。保留现有 G1 运行记录，使用新的唯一证据目录。

工作区 `/home/wq/whut_thesis/expv3_multi_virtual_a`，分支 `feature/virtual-rigid-formation-a`，基线 `567bdecba9676313da3bc9a97b82f57b5b569a2b`。

## 问题与数学合同

`formation_continuous_a/geometry.py` 的实际复合 Simpson 积分使用相邻两个网格步长：若网格步长为 `d=h/n`、`n` 为偶数，则一个完整 Simpson 面板宽度是 `2d`，共 `n/2` 个完整面板。

`bounds.py` 的 `simpson_panel_remainder(H,M4)` 定义为每坐标 `H^5*M4/2880`，但当前 `position_budget` 将 `h/n` 作为面板宽度并乘以 `n+1`。对完整面板，正确的保守总余项至少应包含：

```
(n/2) * (2*d)^5 * M4 / 2880
```

再单独加入任意站点查询时最后一个不完整区间的可证明余项。请对照 `state_at_s` 实际局部积分方式确定不完整区间的最大面板宽度和数量，不要只套用完整面板公式。将各段每坐标上界相加，再保守合成为二维位置余项；继续计入浮点、角度和三角函数误差保护。

上述问题是**预算可能低估**，不表示已证明当前轨迹数值错误。不能通过放宽 `1e-7 m` 预算、删检查或将采样误差称为连续域证明来通过。

## 允许的修改

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/bounds.py`：修正面板计数、宽度和局部余项。
- 同目录 `geometry.py`：若面板细分、局部查询或 refinement 必须随之调整，可做必要修改；保持既有公开接口和 C2 合同。
- `tests/test_virtual_a_profile.py` 及必要的 G1 benchmark：增加针对本问题的测试和证据，不修改原有测试预期以掩盖失败。
- 新的 `evaluation/virtual_formation_a/G1/<返工run-id>/` 证据。

## 测试要求

1. 用手算参数检查单个完整 Simpson 面板和多面板预算，直接验证采用 `2d`；覆盖多个偶数 `n`，包括 `n=2` 和较大 `n`。
2. 测试落在完整面板内部、奇数网格节点和曲率段边界附近的任意 `s` 查询，说明实际局部积分面板的余项界。
3. 构造具有非零四阶导数的合法 C2 曲率路径，用独立高精度积分核对多处位置误差均不超过报告的总预算；同时测试 refinement 和超预算时 `BOUND_UNCONFIRMED`。
4. 重跑 `test_virtual_a_geometry.py`、`test_virtual_a_profile.py`、G1 双移线/镜像/`L=0.5 m` 前车超限 benchmark，以及任务书要求的安全 geometry 回归。旧测试输出继续重定向。
5. 保留原始日志、实际命令和退出码；记录完整面板余项、局部余项、浮点保护、总预算、面板数和与独立积分的误差。记录新代码与旧证据的哈希。

参考命令，`VA_RUN` 须指向新的唯一目录：

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v
python3 -B tools/virtual_a_benchmark.py --stage G1 --output "$VA_RUN/benchmark"
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression geometry
```

## 验收与停止

- 完整面板和局部查询的余项公式均与实际积分一致，可由 Codex 独立重算；误差预算满足原合同或明确拒绝，不能静默低估。
- 原 G1 刚性、端点、航向、曲率、C2、镜像及负例标准继续通过。
- 保护清单表明旧算法、配置和历史证据未变；新 run 记录真实结果，不覆盖旧 run。
- 输出 `CLAUDE_REPORT.md`、命令与日志、测试结果、预算与独立积分对照、源码及证据哈希。完成后停止，等待 Codex 独立复验；不得自动进入 G2。
