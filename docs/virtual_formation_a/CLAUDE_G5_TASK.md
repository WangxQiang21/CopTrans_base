# Claude G5：新旧后端并存的ROS/RViz交互与权威同步回放

## 0. 前置、研究边界与执行限制

仅用户明确下达G5，且G0–G4有Codex ACCEPTED、源码/证据未漂移后执行。正式基线祖先`567bdecba9676313da3bc9a97b82f57b5b569a2b`，在Codex准备的`feature/virtual-rigid-formation-a`隔离worktree。不得操作旧工作区、提交/推送，完成即停止。

三车offset `1:(1,0),2:(0,1),3:(0,-1)m`，车体.738×.500m；统一A正向，F沿psi，初始静止/平行/直行A。默认FREE，回直STRAIGHT是显式实验选项。旧FR06 `uniform_v02`及legacy必须保留。无控制、CAN、Gazebo、在线抢占或动态重规划。

本阶段复用现有三车基础设施，将G4已审新模型接到真实ROS交互；不能把播放看起来合理当作几何/动态/碰撞认证。

## 1. 阶段目标和文件白名单

新增：

- `P/formation_continuous_a/ros_adapter.py`：无ROS的后端请求、投影字段、信封验证、结果摘要hook。
- `tests/test_virtual_a_ros_adapter.py`、`tests/test_virtual_a_playback.py`。
- `tests/ros_virtual_a_smoke.py`：真实私有master交互验证、保存完整信封和实际状态。

允许小范围修改：

1. `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`：保留旧注入接口/默认行为，加入新hook分支。
2. `P/formation_playback.py`：新增有限版本分派及新权威模型帧求值，保留旧函数语义。
3. `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`：新后端接收、模型验证、反馈身份和清理。
4. `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py`：backend/admission兼容，保留真实车头和手工重置语义。
5. `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`、`run_demo.sh`：新增显式后端选择/新参数，不改变旧默认。
6. 优化器`CMakeLists.txt/setup.py`：仅必要安装登记。优先仍运行原节点脚本，不复制新1200行节点。
7. 新包`pipeline/trajectory/contracts`仅真实接入所需的小范围适配，benchmark可增加G5准备/汇总入口；新证据目录。

`P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

不改消息字段/MD5、原`transport_formation.yaml`、旧uniform核心、旧测试预期、旧地图和历史证据。共享接入修改不能挪作旧算法重构；超白名单先报告Codex。

## 2. 已有真实调用点与最小hook设计

已有节点：`UniformPlannerNode.__init__(plan=plan_uniform)`、`on_request:630`、`_publication_times:551`、`_finish_failure:887`、`_finish_success:918`、`on_playback_status:1070`。源码当前直接构造UniformPlanningRequest、使用plan.phases、旧Projection和版本常量，不能只换solve函数。

保留原签名位置参数及默认计划注入，新增keyword-only或兼容尾参数`backend_hooks=None`。原测试`UniformPlannerNode(plan=fake)`必须仍走旧路径。仅选择新backend时由main创建`VirtualABackendHooks`，hooks覆盖：

```python
class VirtualABackendHooks:
    backend = 'virtual_rigid_a'
    model_version = 'virtual_rigid_a.timed.v1'
    semantic_version = 'virtual_rigid_a.playback.v1'
    def request_from_snapshot(self, snapshot, grid, settings, initial_context): ...
    def solve(self, request) -> VirtualAResult: ...
    def publication_fields(self, snapshot, result, stamp, dt) -> dict: ...
    def verify_envelope_fields(self, envelope_fields) -> VerifiedPlayback: ...
    def result_summary(self, result) -> dict: ...
```

不建立动态插件注册平台。旧分支可保留原代码，新分支仅在请求构造、publication_times/投影/成功信封验证、结果摘要和版本字符串调用点分派。不要给新计划伪造旧phases/UniformSampleProjection或塞进uniform_plan来绕过类型限制。

共享状态机继续处理串行admission、map版本变化、generation过期结果、busy、发布后的PLAYING/HOLD_END反馈和超时。新规划watchdog依据新60s软预算加明确容差设置，不能沿用旧30+15而在合法新求解中途提前发布失败；超时结果即使后来返回也不能发布。实际库调用不可中断时保持单请求占用直到退出，不能同时启动第二个求解线程假装已抢占。

## 3. 初态来源与自由终点续接：必须落地

旧FormationPlanRequest不携带速度/模式/轮系。新后端不得仅因车头平行就断言轮系直行，也不得从FREE终点直接假设回正。

### 3.1 明确实验重置和普通续接

- 首次新后端请求须在RViz做2D Pose Estimate建立新虚拟实验。原uniform_v02的自动fallback行为保持不变，不为新后端声称读到了真实轮系。
- 新后端在原节点内增加只对virtual_rigid_a启用的`/initialpose`观察回调：在非busy时校验frame/姿态、保存本进程reset context和对应F位姿；它代表用户显式重置虚拟实验，声明A直行静止，不是物理测量。
- 在原`/formation/job_state`JSON可附`initial_reset_ready/context_id`，便于测试/前端确认事件已处理后再发goal；不新增消息类型/通用事件系统，不借Header.seq传业务状态。
- request适配要求匹配已确认reset的F位姿，真实车辆车头由原bridge读取/生成后逐车验证，不覆盖。初态source=`rviz_reset`并把本地context_id写入新请求/模型摘要。
- 普通目标衔接使用本后端保留的权威终态，只有全车静止、theta_i≈psi且三车理想A轮系均直行时接受source=`verified_terminal`。须检查C_i(T)或理想轮角，不仅检查kappa_F(T)。例如kappa_F(T)=0而eta(T)非零时，前车轮系仍可能非直行。
- 非兼容FREE终态后，普通goal明确拒绝`UNSUPPORTED_INITIAL_STATE`，保留实际静止终态和说明；用户重新2D Pose Estimate后才按新实验声明接受。不得插S或在hold帧修改车头/轮系。

这是单会话内的有限实验状态，非在线控制状态估计。若goal回调先于reset观察完成，拒绝`INITIAL_RESET_NOT_CONFIRMED`并允许稍后重试，不在证据不足时推测已重置；不悄悄缓存多请求队列。

### 3.2 目标策略适配

新launch参数`virtual_a_goal_policy:=free|straight`，默认free。FREE接受原消息全车goal_heading_free=true；STRAIGHT为用户显式附加kT=etaT=0。固定/混合目标车头首版不支持，除非其信息与显式STRAIGHT完全相容，否则明确拒绝，不能静默改原请求。

YAML `allow_reverse:true`只是许可，不强迫倒车；新后端有效能力写forward-only而不改全局配置。初始heading_policy若输出不平行，则明确不兼容，不改旧配置求通过。

## 4. 权威信封、投影和播放接口

复用现有`FormationPlaybackPlan`字段：header/plan_id/formation_id/backend/semantic_version/model_version/model_sha256/model_json/certification/trajectory。字段和MD5不改，新模型JSON含原请求、初态source/context、实际限制、grid摘要及G3完整曲率/T/数值版本。

request_sha256沿用物理请求含义，planning_context_sha256额外绑定地图及实际规划/时间/碰撞设置，进入完整模型和发布审核报告；不能把G2的request摘要到G5重新解释为另一套字段。先冻结上下文、构造模型和审核，再投影和发布，避免加grid后模型摘要变化。

模型核心不嵌入自指摘要或自证passed；审核报告单独绑定最终model_sha，在新质量报告中保存。`certification='virtual_a_geometry_dynamics_collision/v1'`只在G4完整审核通过时发送，并且它只是标签。发布前需要真实三层审核，接收方重新检查结构/几何/动态、原请求身份和样本绑定，不能只信标签。

新增无ROS分派：

```python
def verify_playback_plan(*, backend, **fields):
    # uniform_v02 -> 原verify_uniform_playback_plan（不改其版本语义）
    # virtual_rigid_a -> 新verify_virtual_a_playback_plan
    # 其它 -> 明确拒绝

def verify_virtual_a_playback_plan(*, plan_id, formation_id, backend,
    semantic_version, model_version, model_sha256, model_json,
    certification, trajectory_fields, expected_request=None): ...
```

播放端保留已收到并规范化的原始`/formation/plan_request`快照，按plan/formation身份和admission记录匹配；对原始start/goal/真实headings/形状/策略逐字段独立比对，不能只从model_json重建一个“expected_request”再自证。初态source/context和规划上下文还须与该次已确认job/reset记录相符。缺少匹配原请求、身份记录或地图摘要时拒绝/等待明确重发，不默认为已认证；单个model_json能重建不代表它响应了当前请求。

保留原`FormationPlaybackTrajectory.uniform_plan`用于旧后端；新增独立`virtual_a_plan`或明确typed槽，并在`frame_at`先做新分支。新帧生成旧`PlaybackFrame/PlaybackVehicleState`，直接复用`build_markers`、命名空间和TF发布。所有x/y/theta/v/omega由同一`state_at(t)`恢复，不独立插值三车中心。

投影共享time_from_start和header.stamp。ROS Duration纳秒量化后，以**实际量化时刻**重新求权威状态生成样本/验证，不能先按未量化t求值再只改时间戳。节点验证/构建耗时结束后最后统一restamp两个header，再publish，不能让播放从中途开始。

新模型为单一连续A相位，可均匀dt=.02采样加端点；不伪造旧mode setup边界。零运动单样本、T=0合法。验证每车位置/车头/曲率/中心速度/角速度、模型和request/map/limits绑定。播放端不持有地图时不得声称自己重新跑了地图碰撞，碰撞通过来自发布前匹配该地图的G4审核记录。

## 5. 失败、busy和后端隔离

run_demo后端白名单、launch节点条件、bridge admission、ROS播放backend、反馈身份检查必须一起扩展。默认仍uniform_v02；新选择后旧C++/optimizer或旧uniform求解不同时启动，只使用一个发布者。

- 新合法请求被admit后清旧可执行结果；在没有完整审核前只显示合法静止起点/规划状态。
- 规划失败、审核失败、未知版本、错误摘要、样本篡改、map变更、过期结果不得开始或重启旧动画。
- busy拒绝不消费新start override、不清理当前合法轨迹、不替换active plan身份。
- 播放终态/开始回执必须匹配plan_id、formation_id、backend、model摘要，错误身份不能释放busy或冒用审核标签。
- FREE不兼容续接请求属于明确初态拒绝，不能把显示的旧终态当作该新请求成功。

## 6. 单元、集成与真实ROS验证

### G5-A hook与纯Python

- 新/旧backend构造与默认行为，保持原plan=fake测试可用。
- snapshot到VirtualAMapRequest真实车头/形状/goal策略、初始化source/context正确。
- 非兼容车头、非直行终端轮系（含k_F=0但eta非零）、无reset确认、固定/混合goal语义必须拒绝。
- 信封JSON/模型版本、request/grid/limits/hash、样本x/y/theta/v/omega和纳秒时间篡改负例。
- 至少100个非发布样本时刻的frame与权威state对照；刚性<=1e-6m、状态误差<=1e-7相应单位。
- NO_MOTION、FREE终态保持不回正、STRAIGHT合法续接、busy和错误反馈身份。

### G5-B 新后端真实ROS/RViz

复用G4已验收场景和保存的原请求身份，不用手工模型替代自主求解。新增smoke driver使用`/initialpose`和`/move_base_simple/goal`，不向规划器注入参考/轨迹。

至少完成：

1. 私有master启动新backend，手工/等效标准topic设置起终点，自主规划、完整回放到HOLD_END；至少一条非平凡弯道来自G4已知可行集合。
2. 显式STRAIGHT实验中完成第二个普通goal的合法连续衔接，无重新设置车头；FREE实验核对实际终态，并按其兼容性接受/拒绝续接，具体非直行轮系拒绝由真实构造单元例必测。
3. 一个已admit但无合格候选/目标碰撞的失败请求：旧动画停止且不误播，随后显式重置新实验能够恢复。
4. 播放中goal和initialpose的busy拒绝，不能消费override或中断当前合法计划。
5. 捕获真实完整信封，在接收端进行未知版本/错误摘要/位置/车头/速度/时间篡改，均拒绝并按身份规则清理；这些是播放负例，不算自主规划成功。
6. 比较TF、Marker和状态中的同一实际elapsed_s与权威model；墙钟采集延迟单列，不能把时间错位当几何误差，也不能放宽模型本身1e-7一致标准。保存至少起点/弯道中部/终点图像或完整视频，图像不替代数值记录。

driver自行保存`--output`指定新目录的report、信封、原请求、状态、TF/Marker采集和清理日志。权限/图形环境不允许启动时标NOT_RUN并停止“真实ROS通过”的结论，不能用mock截图替代。

### G5-C 旧后端回归

通过G0安全runner重定向旧FR01–FR04输出后跑相关FR01–FR05测试；原单车Hermite/时间/播放受影响测试；原FR06参考复审和至少一次2.8m无seed自主搜索，新结果只写新run目录。

私有master单独启动`uniform_v02`并运行原`ros_uniform_playback_smoke.py`的main/failure/tamper，保存新输出。不可在同一master同时启动两个后端，也不修改旧消息/阈值让新接口过关。

## 7. 开发步骤和执行命令模板

G5-A先无ROS hook和frame，纯单元/旧反馈测试通过；G5-B再扩展bridge/launch/backend与新真实smoke；G5-C最后旧后端实际回归。不要一次复制整节点后才试运行。

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G5/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
set -o pipefail
python3 -B -m unittest discover -s tests -p 'test_virtual_a_ros_adapter.py' -v 2>&1 | tee "$VA_RUN/logs/adapter.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_playback.py' -v 2>&1 | tee "$VA_RUN/logs/playback.log"
python3 -B -m unittest discover -s tests -p 'test_uniform_feedback_identity.py' -v 2>&1 | tee "$VA_RUN/logs/old_feedback.log"
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression full
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash
cd ..
```

`VA_G4_MANIFEST`和`VA_MAP_DIR`必须从实际Codex ACCEPTED的G4证据定位，不能猜某个latest或用手工新地图替换。检查其中cases的map_dir/sha与真实PGM一致后再填实际绝对路径。

新后端独立终端启动示例（端口先确认未占用，保存并仅清理本任务创建的进程）：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$PWD/$VA_RUN/ros_new"
roscore -p 11585
# 另一个已source相同环境、相同ROS_MASTER_URI的终端：
./run_demo.sh --map "$VA_MAP_DIR" --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true virtual_a_goal_policy:=straight
# 测试终端，VA_G4_MANIFEST为实际已核验文件：
python3 -B tests/ros_virtual_a_smoke.py --scene-manifest "$VA_G4_MANIFEST" --scenario main --output "$VA_RUN/ros_main/report.json"
```

新driver另提供failure/tamper；FREE实验另一次显式启动free，不动态偷换正在执行请求的策略。旧后端换独立master或关闭本次节点后再启动，不运行全局`pkill/rosnode kill -a`。实际最终命令、环境和PID写入COMMANDS.md，不能把模板当已执行证据。

## 8. 交付、量化标准和独立验收

- 新后端至少一条真实非平凡自主路径完整回放，初末及所有采集时刻同一权威模型；刚性<=1e-6m、模型状态对照<=1e-7、终点<=.005m/.2°，纳秒时间语义一致。
- 新/旧串行状态与全部身份/失败负例正确，规划失败不可误播旧计划。
- 新后端全程A前向，内部曲率节点不停，FREE终态不隐式回直；旧FR06混合模式仍按原合同工作。
- 没有未说明的NOT_RUN作为必过项；共享接入相关旧回归和至少一次旧真实ROS检查完成。真实ROS被阻塞时只能部分完成，不能ACCEPTED。

交付差异说明/函数hook清单、build日志、单元与旧回归结果、实际新旧ROS信封/请求/地图/hash、PLAYING→HOLD_END/busy释放/失败清理记录、TF/Marker与模型时间对照、图像或视频、保护哈希、全部失败和`CLAUDE_REPORT.md`。不改旧验收JSON，只在新证据目录形成对照。

Codex独立：审阅共享差异是否限于hook和版本分派；检查旧默认/消息MD5/命名空间；从真实信封重建模型并对非样本t求值；检查request来源和初态轮系继承；自己触发至少一条真实新请求及失败/身份负例；核查旧后端关键回归和历史保护。不能仅相信driver summary或视频。

**停止：完成G5后等待Codex最终验收。不得继续做控制、实车、时间最优或论文创新声明，不提交/推送Git。**
