# C1：精确LOS及二维分离超平面凸走廊

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

## 前置与文件

C0 ACCEPTED。只证明参考点几何自由，不接NLP。读P/pipeline.py `_astar_route/make_route_guides/_cell_of`、contracts.py GuideRoute/RouteGuideConfig、collision.py load_map及formation_collision.py的地图与SAT。

拟新增P/route_processing.py（LOS/简化）、P/corridor.py（凸几何和独立审核）、tests/test_virtual_a_corridor.py；contracts.py加两个简洁数据类。禁止修改optimizer、timing、审核器和ROS配置；可视化由V1单独做。

## 接口、公式和步骤

1. `segment_is_free(grid,p0,p1,margin_m=0.0)->bool`拟新增：supercover或bbox枚举原闭栅格方块，精确segment-AABB，unknown/地图外为障碍，触边拒绝。零段按点；距离场不作认证。
2. `simplify_route(grid,route:GuideRoute)->GuideRoute`：最远可视点贪心，保留端点/source/grid；先验原A*相邻段，切对角障碍即拒绝该路线，不声称A*天然合法。
3. `ConvexCorridor(seed_start,seed_end,A,b,vertices)`（单位法向/CCW/m）、`CorridorChain(regions,route_source,grid_sha256)`；无需复制plan字段，ROS发送时绑定请求。`build_corridor_chain(grid,guide,*,box_padding_m=2.0)`和`audit_corridor_chain(grid,chain)->dict`拟新增。
4. B=seed bbox扩2m，裁地图内缩1e-8。覆盖B及安全距离膨胀范围内全部occupied cell，不按未经证明的附近距离跳过。segment-cell先相交测试，再端点到方块边/方块顶点到segment投影求最近点对。n=(o−s)/dist，h=max端点n投影，l=min四角n投影；gap=l−h>0。ρ=min(.05m,min_gap/4)，d_safe=0仅参考点几何自由；β=(h+ρ+l)/2−1e-8，逐平面裁B。不得跨自由通道取障碍凸包。
5. C内含种子、有正面积。共同种子端点附近B保留小圆盘；portal相交，Chebyshev LP max r s.t. A x+||Arow||r≤b，scipy.optimize.linprog。r≥min(.005m,.1 resolution)、area>1e-10m²，seed违约≤1e-9m。退化则CORRIDOR_DEGENERATE，不宣称物理无解；最多对问题seed中点切分一次，不无限递归。
6. 独立审核用polygon-cell SAT遍历多边形bbox所有原始occupied cell（含触边保守阈值），不依赖生成超平面证明；另核地图包含、seed、portal。结果passed/reason/最大违约/最小r/面积/首碰cell；无三车FEASIBLE字段。

## 测试与验收

小grid至少7类：空场、单盒、L角、凹U、窄缝、边界、点接触portal；附穿cell角LOS反例、漏选障碍侵入B反例、seed触障碍反例。所有输出链独立审核100%通过，退化允许明确失败。相同输入两次CCW规范顶点和摘要一致。记录50×50样例耗时，暂不凭空承诺性能数值，须进入请求预算。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```
首项拟新增，后项现有含真实求解，确认输出重定向再单次运行。保存一份grid/route/A,b/vertices/portal/SAT JSON供V1消费。Codex手算单盒平面、独立SAT一个完整C。完成几何门后停，不提前接入曲率规划。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
