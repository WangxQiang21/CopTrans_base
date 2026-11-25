"""Render the short qualification summary from the same measurements as the gates."""
from pathlib import Path
from qualify import REPORT, load, quant


def number(x, digits=3):
    return "待测" if x is None else f"{x:.{digits}f}"


def historical_summary(data):
    from route_evidence import HISTORICAL
    from collections import Counter
    historical=data.get("historical_closed_loop") or {}
    refs=historical.get("references") or {}
    trials=historical.get("trials") or []
    offline=data.get("historical_offline") or {}
    indexed={(r["archive_id"],r["task"],r["profile"]):r for r in offline.get("cases",[])}
    text=["### 历史长途复测",
          "直距取实际选定的起终点，实走里程按单车里程计或三车货物参考点的连续采样累加。表内里程为米，耗时为秒，挂点误差为毫米。失败轮保留已走里程。"]
    lines=["| 历史编号 | 地图 SHA 前八位 | 起点 → 终点（x, y, yaw） | 直距 | 旧实走 | 旧结果 |",
           "|---|---|---|---:|---:|---|"]
    curves=["| 历史编号 | 闭环试次 | 结果 | 实走里程 | 规划耗时 | 计划时长 | 最大挂点误差 | 计划切换 | 实测切换（各车） |",
            "|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for family,source in HISTORICAL.items():
        old=refs.get(family)
        if not old: continue
        sig=old["signature"]
        fmt=lambda coords: "("+", ".join(number(x,3) for x in coords)+")"
        lines.append(f"| {source} | {sig['map_sha256'][:8]} | {fmt(sig['start'])} → {fmt(sig['goal'])} | {number(old['chord_m'],2)} | {number(old['motion']['arc_m'],2)} | {old['status']} · {old['reason']} |")
        original=(indexed.get((source,sig['task'],sig['profile'])) or {}).get('source_plan') or {}
        old_switch="/".join(str(v) for _,v in sorted(original.get('sampled_switches_by_vehicle',{}).items())) or '待测'
        old_mount=old.get('mount_m')
        def counts(switches):
            return '/'.join(str(v) for _,v in sorted(switches.items())) or '待测'
        curves.append(f"| {source} | 历史三车 standard | {old['status']} | {number(old['motion']['arc_m'],2)} | {number(old.get('planner_wall_s'))} | {number(old.get('plan_duration_s'),2)} | {number(1000*old_mount if old_mount is not None else None,2)} | {old_switch} | {counts(old['motion']['sampled_feedback_switches'])} |")
        for r in trials:
            if r['family']!=family or not r['same_closed_loop_conditions']: continue
            m=r.get('mount_m')
            curves.append(f"| {source} | {r['run_id']} | {r['status']} · {r.get('reason')} | {number(r['observed_m'],2)} | {number(r['planning_s'])} | {number(r['duration_s'],2)} | {number(1000*m if m is not None else None,2)} | {r['planned_mode_switches'] if r.get('planned_mode_switches') is not None else '待测'} | {counts(r['sampled_feedback_switches'])} |")
    text.append("\n".join(lines))
    text.append("下表逐次列出同图、同起终点、同载荷几何、同物理参数和同速度档的三车运行。旧计划三列切换数按三辆车的 CDR 采样模式分别统计，新计划按共同运动阶段统计。实测切换按各车已记录的模式反馈统计，采样间隔以内的瞬时切换可能漏计。")
    text.append("\n".join(curves))
    if indexed:
        bins=Counter(r['status'] for r in indexed.values())
        text.append("冻结历史离线请求的独立审计已记录 "+str(len(indexed))+" 次，结果为 "+"、".join(f"{k} {v} 次" for k,v in sorted(bins.items()))+"。单车和 baseline 为新增复测，旧归档只提供三车 standard 原始对照。逐条 SHA、状态和失败原因见 [measurements.json](measurements.json)。")
    prior=data.get("short_regression") or []
    if prior:
        campaign=prior[0].get("campaign","历史试验")
        completed=sum(r["status"]=="passed" for r in prior)
        expected_stops=sum(r["family"]=="fault" and r["status"]=="stopped" for r in prior)
        unexpected=[r for r in prior if r["status"]!=("stopped" if r["family"]=="fault" else "passed")]
        text.append(f"保留 `{campaign}` 的 {len(prior)} 个 GUI 试次，其中 {completed} 次正常到达、"
                    f"{expected_stops} 次注入故障后停车、{len(unexpected)} 次未达到该轮预期。"
                    "正式闭环另计 96 个 GUI 试次及 4 个无界面试次。")
        for run in unexpected:
            text.append(f"`{run['run_id']}`（{run['task']} / {run['profile']} / {run['family']}）"
                        f"结果为 `{run['status']}`，原因 `{run['reason']}`。")
    incident=data.get("transport_incident") or {}
    if incident.get("origin_cause")=="host_wlo1_link_timeout_correlated_with_gazebo_transport_loss":
        text.append(f"`{incident['run_id']}` 的 Gazebo 物理里程计仍在推进，ROS 仿真时钟和反馈在 Wi-Fi 链路超时后停止送达。"
                    "三车的控制时钟看门狗触发并发送零速度指令。现有记录只能核验发送，无法核验 Gazebo 收到指令或实际停稳。"
                    "旧试次作为非预期停止归档，正式矩阵在本机回环网络上重新独立采集。"
                    "宿主事件、三车时间轴和原始文件摘要见 [measurements.json](measurements.json)。")
    return "\n\n".join(text)


def write_summary(out,result,data):
    cohorts={n for group in (data.get("legacy_summary") or {}).get("cohorts",{}).values()
             for n in group.get("cohorts",[])}
    if cohorts=={4}:
        template=release_instructions(data["software_defaults"])
    else:
        template=(REPORT/"README.md").read_text().split("## 实测与已知故障")[0]
    gates={r["id"]:r for r in result["checks"]}
    passed=sum(r["status"]=="PASS" for r in result["checks"])
    states={"INCOMPLETE":"尚未完成","COMPLETED":"已通过","BLOCKED":"受阻"}
    sections=[template+"## 实测与已知故障\n",
        f"本次自动验收{states[result['task_state']]}，共 {passed}/{len(gates)} 项通过。机器可读结果见 [acceptance.json](acceptance.json)。"]
    source=data.get("source_manifest") or {}
    if source.get("binary_commit") and source.get("system_commit"):
        sections.append(f"本表的离线求解快照为 `{source['binary_commit'][:12]}`，Gazebo 与控制验证使用系统快照 "
                        f"`{source['system_commit'][:12]}`。原生规划库 SHA 与逐试次身份见 [manifest.json](manifest.json)。")
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
    behavior=load(out/"behavior.json")
    if behavior:
        from publish_behavior import readme_paragraph
        sections.append(readme_paragraph(behavior))
    pruned=gates["MODE_PRUNING"]["measured"]
    sections.append(f"冗余辅助模式专项已从 {sum(v['pairs'] for v in pruned.values())} 个预置序列中完整删除 {sum(v['removed'] for v in pruned.values())} 个。核心集的纯 A 判定要求内部停车次数为零。必要辅助使用窄槽侧向位移上界和编队挂点满秩反例检验。")
    g=data["gazebo"];success=[r for r in g if r["status"]=="passed"]
    mount=[r["metrics"]["max_mount_error_m"]*1000 for r in success if (r.get("metrics") or {}).get("max_mount_error_m") is not None]
    terminal=[v["terminal_position_error_m"]*1000 for r in success for v in r["metrics"].get("vehicles",{}).values() if v.get("terminal_position_error_m") is not None]
    sections.append(f"当前批次有 {len(g)} 轮已归档的真实 Gazebo 闭环，其中 {len(success)} 轮正常完成，{sum(r['family']=='fault' and r['reason']=='external_stop' for r in g)} 轮执行了注入停车。正常完成轮次的最大车辆终点误差为 {number(max(terminal,default=None),2)} mm，最大编队挂点误差为 {number(max(mount,default=None),2)} mm。各场景的全部试次与失败原因保存在逐轮记录中。")
    delivered=[r["plan_behavior"] for r in g if r.get("plan_behavior")]
    if delivered:
        sections.append(f"该批 {len(g)} 个 GUI 请求中，{len(delivered)} 个实际发布了可供复算的权威模型，"
                        f"{sum(m['pure_A'] for m in delivered)} 个为纯 Ackermann。最长连续 Ackermann 段为 "
                        f"{max(m['longest_continuous_A_m'] for m in delivered):.2f} m，"
                        f"各模型内部停车合计 {sum(m['internal_stops'] for m in delivered)} 次。")
    if data.get("historical_closed_loop"):
        sections.append(historical_summary(data))
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
    if data.get("concurrent_gazebo_native_control"):
        stress=data["concurrent_gazebo_native_control"]
        counts=[stress["setups"][k]["deadline_misses"] for k in ("1/1","3/1","3/2","3/3")]
        message=(f"同一原生控制台与 Gazebo 同时运行的额外压力测试中，单车超期 {counts[0]} 次，"
                 f"三车分别超期 {counts[1]}、{counts[2]}、{counts[3]} 次。")
        if stress["misses"]:
            jitters=[m["release_jitter_ns"]*1e-6 for m in stress["misses"]]
            compute=max(m["tracking_compute_ns"]*1e-3 for m in stress["misses"])
            message+=f"超期周期的唤醒延迟为 {number(min(jitters))}–{number(max(jitters))} ms，当次跟踪计算最长 {number(compute)} µs。"
        sections.append(message+"各次超期及原始 CSV 的 SHA-256 见 `measurements.json` 中的 `concurrent_gazebo_native_control`。")
    if data.get("admission"):
        sections.append("短路径 DDS 准入测试每组 5 次预热后正式测 30 次。"+"。".join(f"{names[k.split('/')[0]]} {k.split('/')[1]} 的 P95 为 {number(1000*quant(v,.95),1)} ms" for k,v in data["admission"].items())+"。")
    problems=[r["id"]+" ("+r["status"]+")" for r in result["checks"] if r["status"]!="PASS"]
    if problems: sections.append("待通过门项为 "+"、".join(problems)+"。")
    sections.append("开发期间记录了仿真节点首个时钟错序、三车 S→A 末时刻浮点残差及 DDS 发送回调长尾。原始失败试次保存在本机。旧版单车的采样 Twist 推断模式有歧义，旧版列按同请求的预算内认证结果统计。ASan、UBSan 和实际 C++ 求解器的测试记录见 `BUILD_TESTS_SANITIZERS`。TSan 宿主探针记录为 `"+str((data.get("asan_smoke") or {}).get("tsan") or "未采集")+"`。实车验证状态为 `HARDWARE_NOT_VERIFIED`。")
    (out/"README.md").write_text("\n\n".join(sections)+"\n")


def release_instructions(defaults):
    promoted=all(defaults["planners"].get(k)=="hybrid_joint" for k in ("single","multi"))
    select="" if promoted else " --planner-backend hybrid_joint"
    return f"""# V3 联合规划与闭环验收

## 仿真

从仓库根目录运行以下任一命令。关闭当前 Gazebo 后再开始下一次。

```bash
./scripts/test.sh{select}
./scripts/test.sh --task multi{select}
./scripts/interactive.sh{select}
./scripts/test.sh --task multi --planner-backend legacy
```

交互仿真用 RViz 的 **2D Pose Estimate** 和 **2D Goal Pose** 设置起终点。单车与编队的原生后端分别为 `single_hybrid_joint_cpp` 和 `formation_hybrid_joint_cpp`。默认速度档为 standard，低速测试可加 `--speed-profile baseline`。本机 `reports/runs/<实验编号>/` 保存实际端点、结果、权威轨迹、运行环境及安装指纹。

## 规划和控制

C++17 搜索生成连续 Ackermann、斜行、原地旋转和舵角准备阶段。CasADi C++ 与 Ipopt 调整几何、时长及连接状态。连续证书检查运动学、车体和货物包络。C++ 执行协调器验证各车收到相同权威模型，再向全部跟踪器发出共同起跑许可。每车执行车体 Twist，仿真反馈经里程计闭环回到跟踪器。

## 复算

干净检出可直接复算仓库内的压缩证据。

```bash
python3 scripts/accept_hybrid_v3.py --compact
```

本批使用两档预封存的 cohort 4 清单。重建原生基准、独立参考认证和计时需 ROS 2 Humble、CasADi C++ SDK。冻结源提交见 `manifest.json` 中的 `numeric_and_control_sources.system_commit`。

```bash
raw=$(mktemp -d)
revision=$(python3 -c 'import json; print(json.load(open("reports/hybrid_v3/manifest.json"))["numeric_and_control_sources"]["system_commit"])')
bash scripts/hybrid_v3/build_snapshot.sh "$raw/system" "$revision"
bash scripts/hybrid_v3/full_benchmark.sh "$raw/system" "$raw/batch" "$raw/witness" "$raw/binary" 4
```

本机发布前，用 `scripts/hybrid_v3/release_e7.example.json` 填入同一冻结候选的实际目录，并运行 `python3 scripts/hybrid_v3/release_e7.py --config <文件>` 检查工件。加 `--stage <新目录> --packaging-ref <提交>` 后，程序依次核对全部非默认门项、在隔离检出中运行三个真实默认入口、从干净检出复算，通过后输出本地 Git bundle。实验日志保存在本机。

每个默认入口记录实际加载的本机安装指纹。独立复编时，日志宏里的源码路径映射为冻结路径。同一批目标文件分别按实际目录和冻结目录的 CMake 链接路径重建，两次都与对应安装文件逐字节核对。五个原生文件的实际、冻结和重建 SHA-256 可在 `measurements.json` 的 `default_relocation` 中复查。

"""
