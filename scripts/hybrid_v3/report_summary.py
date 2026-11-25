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
    historical=(data.get("legacy_summary") or {}).get("cohorts",{})
    if historical:
        lines=["旧版在完全相同的核心请求上测得的政策合规 P95 如下。每格对应 5 次预热及 30 次正式请求。门槛内成功数不足以观测 P95 时单独标出。",
               "| 任务 | 速度档 | 简单直线（秒） | 困难多弯（秒） |","|---|---|---:|---:|"]
        for key in sorted(historical):
            task,profile=key.split("_")
            timing=historical[key].get("timing",{})
            def legacy_value(family):
                t=timing.get(family,{})
                if not t.get("complete"): return "待测"
                return number(t["policy_compliant_p95_s"]) if t["policy_compliant_p95_observed"] else f"未达到（{t['certified_in_budget']}/30）"
            lines.append(f"| {names[task]} | {profile} | {legacy_value('C01')} | {legacy_value('C06')} |")
        sections.append(lines[0]+"\n\n"+"\n".join(lines[1:]))
    pruned=gates["MODE_PRUNING"]["measured"]
    sections.append(f"冗余辅助模式专项已从 {sum(v['pairs'] for v in pruned.values())} 个预置序列中完整删除 {sum(v['removed'] for v in pruned.values())} 个。核心集的纯 A 判定要求内部停车次数为零。必要辅助使用窄槽侧向位移上界和编队挂点满秩反例检验。")
    g=data["gazebo"];success=[r for r in g if r["status"]=="passed"]
    mount=[r["metrics"]["max_mount_error_m"]*1000 for r in success if (r.get("metrics") or {}).get("max_mount_error_m") is not None]
    terminal=[v["terminal_position_error_m"]*1000 for r in success for v in r["metrics"].get("vehicles",{}).values() if v.get("terminal_position_error_m") is not None]
    sections.append(f"当前批次有 {len(g)} 轮已归档的真实 Gazebo 闭环，其中 {len(success)} 轮正常完成，{sum(r['family']=='fault' and r['reason']=='external_stop' for r in g)} 轮执行了注入停车。正常完成轮次的最大车辆终点误差为 {number(max(terminal,default=None),2)} mm，最大编队挂点误差为 {number(max(mount,default=None),2)} mm。各场景的全部试次与失败原因保存在逐轮记录中。")
    for paired in data.get("paired_closed_loop",[]):
        old=paired["reference"];matches=paired["matches"]
        if not matches: continue
        endpoint=max(r["terminal_m"] for r in matches)
        body=old["signature"][0]
        text=(f"与旧版实验 `{old['run_id']}` 同地图、起终点、几何及速度档的{names[body]}实验共 {len(matches)} 轮。"
              f"最大车辆终点位置误差由 {old['terminal_m']*1000:.2f} mm 变为 {endpoint*1000:.2f} mm（新批次最差轮次）。")
        if old["mount_m"] is not None and all(r["mount_m"] is not None for r in matches):
            text+=f"最大挂点误差由 {old['mount_m']*1000:.2f} mm 变为 {max(r['mount_m'] for r in matches)*1000:.2f} mm。"
        sections.append(text)
    if success:
        table=["| 任务及速度档 | 样本 | 初值 | 结构编辑 | NLP 构图 | NLP 求解 | 独立认证 | 模型编码 | 显示轨迹采样 | DDS 发布调用 |",
               "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
        fields=("seed_s","edit_s","nlp_build_s","nlp_solve_s","certify_s","encode_s","sample_s","publish_call_s")
        for task in ("single","formation"):
            for profile in ("standard","baseline"):
                stages=[r["publication_event"].get("planning_stages",{}) for r in success
                        if r["task"]==task and r["profile"]==profile]
                observed=[s for s in stages if all(k in s for k in fields)]
                if observed:
                    values=" | ".join(number(1000*quant([s[k] for s in observed],.95),2) for k in fields)
                    table.append(f"| {names[task]} {profile} | {len(observed)} | {values} |")
        if len(table)>2:
            sections.append("正常 Gazebo 试次的分阶段墙钟 P95，单位为毫秒。构图、求解和认证按每次请求内的全部候选累积，编辑可能嵌在候选处理内。各列分别取分位数。\n\n"+"\n".join(table))
        all_clocks=[clock for r in g for clock in r["clocks"]]
        if all_clocks:
            polls=[c["max_consumption_ms"] for c in all_clocks if c["max_consumption_ms"] is not None]
            factors=[c["real_time_factor"] for c in all_clocks if c["real_time_factor"] is not None]
            sections.append(f"真实 GUI 下共 {sum(c['ticks'] for c in all_clocks)} 个车次跟踪周期，"
                            f"最大回调 {number(max(c['max_callback_ms'] for c in all_clocks))} ms。"
                            f"按消息值和时间作唯一匹配的 {sum(c['matched_commands'] for c in all_clocks)} 个命令中，"
                            f"最慢从控制节点提交到 Gazebo 消费用了 {number(max(polls,default=None))} ms。"
                            f"各次跟踪的实时因子为 {number(min(factors,default=None))}–{number(max(factors,default=None))}。")
    real_delays=[1000*a["delivery_and_admission_wall_s"] for r in success for a in r.get("admissions",[])
                 if a.get("delivery_and_admission_wall_s") is not None]
    if real_delays:
        sections.append(f"实际闭环还测得 {len(real_delays)} 个从权威模型打包到整组准入的样本，P95 {number(quant(real_delays,.95),1)} ms、最大 {number(max(real_delays),1)} ms。该数据包含模型大小与 ROS 调度的影响，单独于固定短路径基准报告。")
    if data.get("control"):
        ctrl=data["control"].values()
        sections.append(f"独立 25 Hz 负载台记录每车 {min(r['ticks'] for r in ctrl)} 至 {max(r['ticks'] for r in ctrl)} 个周期，截止违约合计 {sum(r['deadline_misses'] for r in ctrl)} 次。各车 P99 跟踪计算最大值 {number(max(r['p99_compute_ms'] for r in ctrl))} ms，最大回调 {number(max(r['max_callback_ms'] for r in ctrl))} ms。")
    if data.get("admission"):
        sections.append("短路径 DDS 准入测试每组 5 次预热后正式测 30 次。"+"。".join(f"{names[k.split('/')[0]]} {k.split('/')[1]} 的 P95 为 {number(1000*quant(v,.95),1)} ms" for k,v in data["admission"].items())+"。")
    problems=[r["id"]+" ("+r["status"]+")" for r in result["checks"] if r["status"]!="PASS"]
    if problems: sections.append("待通过门项为 "+"、".join(problems)+"。")
    sections.append("开发期间真实复现过启动时两个节点的仿真时钟错序、接收端尚未收到首个时钟、三车 S→A 的末时刻浮点残差、DDS 发送回调长尾。失败试次在本机原始目录保留。旧版单车用采样 Twist 反推运动模式有歧义，表中旧版列仅统计同请求下的认证成功数。TSan 在本机进入测试主函数前因内存映射失败，ASan、UBSan 以及主动执行求解器的测试结果单独列出。硬件状态为 `HARDWARE_NOT_VERIFIED`。")
    (out/"README.md").write_text("\n\n".join(sections)+"\n")
