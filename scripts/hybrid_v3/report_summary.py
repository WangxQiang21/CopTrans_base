"""Render the short qualification summary from the same measurements as the gates."""
from pathlib import Path
from qualify import REPORT, quant


def number(x, digits=3):
    return "待测" if x is None else f"{x:.{digits}f}"


def write_summary(out,result,data):
    template=(REPORT/"README.md").read_text().split("## 实测与已知故障")[0]
    gates={r["id"]:r for r in result["checks"]}
    passed=sum(r["status"]=="PASS" for r in result["checks"])
    states={"INCOMPLETE":"尚未完成","COMPLETED":"已通过","BLOCKED":"受阻"}
    sections=[template+"## 实测与已知故障\n",
        f"本次自动验收{states[result['task_state']]}，共 {passed}/{len(gates)} 项通过。机器可读结果见 [acceptance.json](acceptance.json)。"]
    defaults=data["software_defaults"]["planners"]
    sections.append(f"当前无参入口的单车规划器为 `{defaults['single']}`，三车为 `{defaults['multi']}`，跟踪器为 `{data['software_defaults']['tracking_implementation']}`。")
    names={"single":"单车","formation":"三车"}
    table=["| 任务 | 速度档 | 核心可行通过 | 留出可行通过 | 留出纯 A | 旧版同限制可行通过 | 易 / 难规划 P95（秒） |",
           "|---|---|---:|---:|---:|---:|---:|"]
    for key,m in gates["HOLDOUT_GENERALIZATION"]["measured"].items():
        task,profile=key.split("/")
        core=gates["A_WITNESS_CORE"]["measured"][key]
        timing=gates["PLANNING_LATENCY"]["measured"]
        pair=" / ".join(number(timing.get(key+"/"+d,{}).get("p95")) for d in ("easy","difficult"))
        table.append(f"| {names[task]} | {profile} | {core['success']}/{core['feasible']} | {m['success']}/{m['feasible']} | {m['pure_A']}/{m['A']} | {m.get('paired_legacy_success',0)}/{m['feasible']} | {pair} |")
    sections.append("\n".join(table))
    pruned=gates["MODE_PRUNING"]["measured"]
    sections.append(f"冗余辅助模式专项已从 {sum(v['pairs'] for v in pruned.values())} 个预置序列中完整删除 {sum(v['removed'] for v in pruned.values())} 个。核心集的纯 A 判定要求内部停车次数为零。必要辅助使用窄槽侧向位移上界和编队挂点满秩反例检验。")
    g=data["gazebo"];success=[r for r in g if r["status"]=="passed"]
    mount=[r["metrics"]["max_mount_error_m"]*1000 for r in success if (r.get("metrics") or {}).get("max_mount_error_m") is not None]
    terminal=[v["terminal_position_error_m"]*1000 for r in success for v in r["metrics"].get("vehicles",{}).values() if v.get("terminal_position_error_m") is not None]
    sections.append(f"当前批次有 {len(g)} 轮已归档的真实 Gazebo 闭环，其中 {len(success)} 轮正常完成，{sum(r['family']=='fault' and r['reason']=='external_stop' for r in g)} 轮执行了注入停车。正常完成轮次的最大车辆终点误差为 {number(max(terminal,default=None),2)} mm，最大编队挂点误差为 {number(max(mount,default=None),2)} mm。各场景的全部试次与失败原因保存在逐轮记录中。")
    if data.get("control"):
        ctrl=data["control"].values()
        sections.append(f"独立 25 Hz 负载台记录每车 {min(r['ticks'] for r in ctrl)} 至 {max(r['ticks'] for r in ctrl)} 个周期，截止违约合计 {sum(r['deadline_misses'] for r in ctrl)} 次。各车 P99 跟踪计算最大值 {number(max(r['p99_compute_ms'] for r in ctrl))} ms，最大回调 {number(max(r['max_callback_ms'] for r in ctrl))} ms。")
    if data.get("admission"):
        sections.append("短路径 DDS 准入测试每组 5 次预热后正式测 30 次。"+"。".join(f"{names[k.split('/')[0]]} {k.split('/')[1]} 的 P95 为 {number(1000*quant(v,.95),1)} ms" for k,v in data["admission"].items())+"。")
    problems=[r["id"]+" ("+r["status"]+")" for r in result["checks"] if r["status"]!="PASS"]
    if problems: sections.append("待通过门项为 "+"、".join(problems)+"。")
    sections.append("开发期间真实复现过启动时两个节点的仿真时钟错序、接收端尚未收到首个时钟、三车 S→A 的末时刻浮点残差、DDS 发送回调长尾。失败试次在本机原始目录保留。旧版单车用采样 Twist 反推运动模式有歧义，表中旧版列仅统计同请求下的认证成功数。TSan 在本机进入测试主函数前因内存映射失败，ASan、UBSan 以及主动执行求解器的测试结果单独列出。硬件状态为 `HARDWARE_NOT_VERIFIED`。")
    (out/"README.md").write_text("\n\n".join(sections)+"\n")
