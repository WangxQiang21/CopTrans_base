# Claude总执行方案

输入需求：`/home/wq/下载/VIRTUAL_A_CORRIDOR_DESIGN_BRIEF_V01.md`和`/home/wq/下载/CODEX_INSTRUCTIONS_TO_GENERATE_CLAUDE_PLAN.md`，本目录已落实必要合同，Claude执行不要求访问下载目录或聊天历史。

## 基线保护、角色和授权

核对HEAD `23fd16227e75a68f03f3ac7483555063c8189313`，分支feature/virtual-rigid-formation-a，核对前工作区干净。本轮只产出Markdown。后续Codex另行准备候选feature/virtual-rigid-a-corridor工作区和baseline manifest，Claude不自行创建worktree、切分支或操作原仓库。开始前读本目录架构/复用矩阵及单阶段任务，核对HEAD为交付基线后代和前置ACCEPTED，已有修改核对归属再开发，禁止覆盖/重置数据。

Claude实现、定向测试、交付真实结果；Codex独立审数学/diff/证据并写新验收记录；用户逐门授权。讨论稿“每阶段独立提交”指独立交付，Git commit/push必须另外得到用户授权，不能自动执行。未提交阶段记录base/current HEAD相同+源码hash，不伪造new SHA。

## 阶段依赖和停止点

| 门 | 前置 | 可独立验收能力 | 后续 |
|---|---|---|---|
| C0-A | 当前HEAD/准备工作区 | 前缀积分/失败摘要/solver语义 | 停等Codex，再C0-B |
| C0-B | C0-A | 同候选积分分解、修复后A组冻结 | C1；未批准预算变更继续阻塞受影响成功声明 |
| V0 | 已有G5 | 跟车标签+标准窗口HUD | 可与C0按不重叠文件顺序交付 |
| C1 | C0 | 精确LOS/自由凸链/真实portal | V1或C2-A |
| V1 | C1 | 实际候选走廊显示 | 不假称NLP已消费 |
| C2-A | C1/C0 | portal QP/安全引导/σ分配 | 停等Codex，再C2-B |
| C2-B | C2-A | 弱引导NLP/连续包含/三层审核 | V2/C3 |
| V2 | C2-B/V1 | A*/中线/认证轨迹分层 | V3 |
| C3 | C2-B | 多通道/初值/一次恢复 | C4按需或C5准备 |
| C4 | C3失败证据+用户批准 | 可选姿态感知筛查 | 无收益/无准入可SKIPPED |
| V3 | V0/V1/V2/C3 | 新旧ROS状态/清理回归 | C5 |
| C5 | C3/V3，C4可跳过 | 公平实验、最终回归和结论边界 | 停止，不自行扩研究路线 |

不自动连续执行阶段；每份TASK可单独输入Claude，无需聊天历史。C0先短数学再3请求、C2先几何再NLP；V阶段分开验收，避免一次同时改算法与UI难定位。先完成C0单一任务再考虑其它门。

## 唯一数据路径

P=`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`。

原VirtualAMapRequest→plan_virtual_a→（legacy_route或corridor显式设置）→原solve_open_path_guided→几何审核→原retime→动态→连续OBB→原VirtualAResult/信封→frame_at/state_at公共回放。新走廊硬模式额外连续包含证书，只是参考点证书，不替代三层审核。设置进PlannerConfig/context哈希；实际链/guide/assignment放既有result.search；一个planning_debug载荷供UI，执行轨迹仍原消息，避免新平行模型。

C1最多新增route_processing.py/corridor.py；C2在已有optimizer/管线内接，不复制求解器；C3替换旧重试次数，不新增外套恢复；C4只筛查；C5独立小实验入口。详见00数学公式及01真实函数签名。

## 简洁性强约束（所有阶段生效）

边界验证一次，内部复用已验证合同；不增无意义wrapper/validator/adapter/manager/factory/registry/额外状态机，不为无证据问题加多级fallback、吞异常、默认假值、重复日志/测试。新增抽象需指向已知错误，否则删。不能删三车曲率/轮系/动态、数值证书、原始地图、连续OBB、模型绑定与失败不误播。只改本阶段白名单，普通工程选择自行处理；核心数学/审核合同变化停止相关开发向用户反馈。

## 测试环境与真实命令规范

仓库根：
```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:${PYTHONPATH:-}"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
```
使用catkin_ws/devel而非猜测根devel；未构建时先纯Python，真实ROS如实NOT_RUN。各TASK标注现有/拟新增测试，新增CLI在完成前不可写成已执行。本轮未运行测试/场景，也不修改测试。

只定向discover；旧测试若写历史，先找到实际输出重定向接口后指向新run。没有接口则选纯函数抽验/后续专门最小测试，不能覆盖历史。旧timing_continuous KeyError如实保留，非本阶段修复目标。

真实ROS用未占用私有master，TASK_V3给出G5实际参数；保持节点/driver同URI，只关闭自己进程，不全局pkill。每阶段不反复整套ROS：V0最小GUI、V3完整关键序列、C5只有代码漂移才补测。solver调用不可立即中断不谎称硬实时。

## 证据和规模

证据统一`evaluation/virtual_a_corridor/<stage>/<UTC-run>/`，一次SUMMARY.md/results.json/commands及必要模型/输入hash和少数图片；只存定位错误所需原始数值，不海量逐点重复。记录diff、源码hash、物理/数值配置、机器/依赖；每门Codex验收文件新建，不改旧验收JSON。

C0三请求单次；C1短几何；C2/C3代表场景单次诊断；C5 12请求×3重复×2组=72次离线，C4未实施不跑C组。不承诺固定成功率提升，不以多算力/宽松审核制造收益。可执行率与真实发布率分开统计。
