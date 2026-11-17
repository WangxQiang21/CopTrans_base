# C4（可选）：按证据准入的三车姿态感知增强

## 独立执行上下文与边界

源码基线`23fd16227e75a68f03f3ac7483555063c8189313`，原分支feature/virtual-rigid-formation-a；只在Codex准备并明确交付的工作区执行。记录HEAD/status/前置ACCEPTED，不覆盖现有数据，不自行建分支、提交或推送。P=`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`。新增接口均为待实现建议，不能假定已存在。

三车偏移(1,0)/(0,1)/(0,-1)m从已有合同取，全车正向统一A，p_i=pF+Rψ r_i，车头独立；保留FREE/STRAIGHT、reset、公共时间律、积分证书、三车运动学/动态、连续OBB和plan/context/model绑定；失败不得误播旧成功。旧FR06和历史证据保护。

**本阶段禁止过度防御性编程**：复用既有模块/类型，输入边界必要验证一次，内部使用已验证合同。不增无意义wrapper/validator/adapter/manager/factory/多层抽象，不加无证据fallback、宽泛吞异常、无限重试、重复日志测试或无关重构。每项新增检查/抽象说明对应哪个可复现错误，否则不加；必要数学、安全、身份检查不能因此删除。

Claude实施时仓库根测试环境：
```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:${PYTHONPATH:-}"
```
若ROS overlay未准备，先纯Python，真实ROS标NOT_RUN。只跑定向测试，禁止全量discover覆盖历史输出。

## 准入、范围和文件

只有C3已验收、至少2个窄通道请求重复出现“F走廊通过而实际OBB失败”，有车体/障碍/κ和σ记录，且一次现有换路线不能解决，经用户明确授权才执行；否则交付SKIPPED即可，不阻塞C5。不能因潜在收益提前实施。

读P/geometry.py map_spatial_state、collision.py audit_collision/error_envelope、corridor.py及optimizer clearance代理。允许corridor.py增加一个筛查函数，pipeline.py调用，tests/test_virtual_a_corridor.py加高价值反例；不改车辆collision核心，不立即把非光滑支持式加入NLP，不开发高维联合规划。

## 最小实施

拟`screen_formation_support(path,chain,allocation,request)->dict`：对有限候选姿态/κ恢复p_i和θ_i，对单位n：n·p_i+Lb/2|n·eθ_i|+Wb/2|n·e_perpθ_i|≤b−d；Lb/Wb从request，各车可以对应不同相邻区域。此式是单一凸区域的充分条件，车辆跨区域接缝会产生假拒绝；首版只作初值优先级/诊断，不能凭失败否定整车可行，也不能把有限采样通过当连续安全。

若支持式过保守，使用已有实际OBB静态筛查直接访问原grid，不逼所有车在同一C。每组少量κ/姿态候选共享原初值上限，不建离散高维网格。初版不改变终审、不改安全余量；d为原车辆审核余量的规划代理而非重复加两次。筛查CPU计入原60s。

## 测试与门槛

矩形四角max n投影与支持式≤1e-12m差；旋转车体、偏移车头不同、车辆跨两个C但原grid安全的反例不得误称不可行。有限点之间碰撞例仍由原连续OBB终审拒绝。至少一个已记录C3失败例经排序改善后在同预算下三层通过，且基准无新安全回归；无收益则默认关闭/保留诊断，不扩成NLP/高维搜索。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collision.py' -v
```
交付准入证据/支持式数值/同请求B对C成本。Codex检查筛查不是最终证书，支持式与四角独立对照；未达到收益门允许SKIPPED并说明，无需返工到强求提升。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
