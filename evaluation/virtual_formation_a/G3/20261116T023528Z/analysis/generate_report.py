"""Generate CLAUDE_REPORT.md from the frozen G3 evidence records.

This is a presentation-only assembler.  It reads results.json, the benchmark
results, the protection diff and environment.json; it does not solve, audit or
change any source.  The manifest binds the resulting report.
"""
import hashlib
import json
from pathlib import Path

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
BENCH = RUN / "benchmark" / "results.json"
SUMMARY = RUN / "benchmark" / "summary.json"
RESULTS = RUN / "results.json"
PROTECTION = RUN / "analysis" / "protection_diff_vs_g2_accepted.json"
ENVIRONMENT = RUN / "environment.json"
OUT = RUN / "CLAUDE_REPORT.md"


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def f(value, digits=12):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "{:.{}g}".format(value, digits)
    return str(value)


def dump_vector(record, side):
    return json.dumps(record[side], separators=(",", ":"))


def main():
    results = load(RESULTS)
    benchmark = load(BENCH)
    summary = load(SUMMARY)
    protection = load(PROTECTION)
    environment = load(ENVIRONMENT)
    cases = results["model_cases"]
    aggregate = results["aggregate"]
    lines = []
    add = lines.append

    add("# G3 公共时间与三车动力学报告")
    add("")
    add("状态：`PASS_PENDING_CODEX_RECHECK`，Codex 接受标记为 "
        "`NOT_RUN_NOT_CLAIMED`。本报告只声明 G3 的公共时间参数化与独立全域"
        "动态审核；不声明地图、碰撞、可执行、ROS/Gazebo、硬件、时间最优或 "
        "G4 已进入。")
    add("")
    add("运行：`{}`；基线 HEAD：`{}`；分支："
        "`feature/virtual-rigid-formation-a`。未提交、未推送、未建分支、"
        "未操作其它 worktree。".format(
            results["run_id"], protection["git"]["baseline_head"]))
    add("")
    add("## 1. 范围与非声明")
    add("")
    add("已验证：")
    add("")
    add("- 整条路径使用同一个五次 smoothstep 时间律 `tau=t/T`，"
        "输出 `s=S*h(tau)` 及 `w,a,j`。")
    add("- 三车完整动力学：位置、速度、加速度、jerk、曲率与曲率导数、"
        "航向角速度/角加速度/角 jerk、横向加速度、轮速与转向角/角速度/"
        "角加速度。")
    add("- 独立动态审核从原始 profile 和 T 重算，不信任 retimer 的峰值；"
        "所有运动实例的全域区间证书通过，节点左右动态和 100 个非均匀"
        "时间反解对照通过。")
    add("- `NO_MOTION` 使用显式 hold 模型：`S=T=0`，只有一个 `t=0` 样本，"
        "不做除法、不做时间反解。")
    add("")
    add("明确未验证/未声明：")
    add("")
    add("- `collision_checked=false`，`executable=false`；没有地图、碰撞、"
        "ROS/Gazebo、硬件、实时或时间最优证据。")
    add("- `DYNAMIC_FEASIBLE` 只是独立全域动态审核结果，不是 Codex "
        "ACCEPTED，也不是可执行轨迹。")
    add("- 未进入 G4；停止等待 Codex ACCEPTED 和用户的 G4 指令。")
    add("")

    add("## 2. 结果摘要")
    add("")
    add("| 项目 | 值 |")
    add("|---|---|")
    add("| 测试 | timing 16 / trajectory 23 / audit 44，全部 OK |")
    add("| benchmark | {}（{} checks，0 failed） |".format(
        summary["status"], len(summary["checks"])))
    add("| 实例 | 9（8 motion + 1 exact NO_MOTION hold） |")
    add("| DYNAMIC_FEASIBLE | {} motion 实例 |".format(
        aggregate["dynamic_feasible_cases"]))
    add("| 短时反例拒绝 | {} |".format(
        aggregate["short_duration_rejections"]))
    add("| 篡改拒绝 | {} |".format(aggregate["tamper_rejections"]))
    add("| 最大实际利用率 | {} |".format(
        f(aggregate["max_actual_utilization"])))
    max_rigidity = max(
        benchmark[case["name"]]["timing"]["geometry_audit"]["metrics"][
            "sample_audit"]["max_pairwise_rigidity_error_m"]
        for case in cases if case["name"] != "NO_MOTION_HOLD")
    add("| 最大几何刚性误差 | {} m（G1 门槛 1e-6） |".format(
        f(max_rigidity)))
    add("| 最大节点状态误差 | {} |".format(
        f(aggregate["max_node_state_error"])))
    add("| 最大节点连续性误差 | {} |".format(
        f(aggregate["max_node_continuity_error"])))
    add("| 最大节点 jump 幅度 | {}（允许的有界 g 不连续） |".format(
        f(aggregate["max_node_jump_magnitude"])))
    add("| 最大时间反解误差 | {} s |".format(
        f(aggregate["max_time_inverse_error_s"])))
    add("| 差分步长/误差 | 见第 8 节 |")
    add("")
    add("环境：Python `{}`，CasADi `{}`，IPOPT 可用 `{}`；`OPENBLAS_"
        "NUM_THREADS=1`；所有证据命令使用 `-B` 和 "
        "`PYTHONDONTWRITEBYTECODE=1`。".format(
            environment["host"]["python_version"],
            environment["solver"]["casadi_version"],
            environment["solver"]["ipopt_plugin_available"]))
    add("")

    add("## 3. 公共时间律")
    add("")
    add("```text")
    add("tau = t/T")
    add("h   = 10*tau^3 - 15*tau^4 + 6*tau^5")
    add("h1  = 30*tau^2*(1-tau)^2")
    add("h2  = 60*tau - 180*tau^2 + 120*tau^3")
    add("h3  = 60 - 360*tau + 360*tau^2")
    add("s=S*h, w=S*h1/T, a=S*h2/T^2, j=S*h3/T^3")
    add("")
    add("速度 v=Q*w, 角速度 omega=H*w")
    add("加速度 p_ss*w^2 + p_s*a, jerk p_sss*w^3 + 3*p_ss*w*a + p_s*j")
    add("yaw_accel=H*a+H_s*w^2, yaw_jerk=H*j+3*H_s*w*a+H_ss*w^3")
    add("lateral_accel=abs(C)*v^2")
    add("wheel: Ew=m^2+n^2, delta=atan2(n,m), wheel_speed=sqrt(Ew)*v")
    add("steer_rate=delta_s*w, steer_accel=delta_ss*w^2+delta_s*a")
    add("```")
    add("")
    add("T 选择：`T0=1 s`，对整段每个 Hermite 段和对应时间区间使用 "
        "G1 外扩区间界，按 1/2/3 阶候选 `(U/limit)^(1/order)` 取最大；"
        "`selected=required*1.01`；最多 8 次 1.05 重缩放；工程预算 600 s。"
        "本运行所有运动实例 `rescalings=0`。")
    add("")

    add("## 4. 实例、模型摘要、请求摘要与 T 选择")
    add("")
    add("| 实例 | 状态 | S (m) | T (s) | required (s) | worst 约束 | 车辆 | 阶 | 模型 SHA-256 | 请求 SHA-256 |")
    add("|---|---|---:|---:|---:|---|---:|---:|---|---|")
    for case in cases:
        sel = case["selection"]
        if sel is None:
            worst = "exact-hold"
            vehicle = "-"
            order = "-"
        else:
            worst = "{} / segment {}".format(
                sel["worst"]["quantity"], sel["worst"]["segment_index"])
            vehicle = str(sel["worst"]["vehicle_id"])
            order = str(sel["worst"]["order"])
        add("| {} | {} | {} | {} | {} | {} | {} | {} | `{}` | `{}` |".format(
            case["name"], case["timing_status"], f(case["S_m"]),
            f(case["T_s"]),
            f(sel["required_duration_s"]) if sel else "0",
            worst, vehicle, order, case["model_sha256"],
            case["request_sha256"]))
    add("")
    add("完整模型 JSON 与请求绑定位于 `benchmark/results.json` 的 "
        "`<case>.model_json`；每个实例的摘要都从该字符串重新计算，"
        "结果见第 11、12 节。所有请求均使用固定偏移 "
        "`ID1=(1,0)`, `ID2=(0,1)`, `ID3=(0,-1)` m，起始 `(0,0,0)`、"
        "航向 0、A 静止直行。")
    add("")

    for case in cases:
        name = case["name"]
        sel = case["selection"]
        add("### {}".format(name))
        add("")
        if sel is None:
            add("精确 `NO_MOTION` hold：`S=T=0`，`selection` 为空；"
                "不使用时间反解或除法。")
            add("")
            continue
        add("`required={} s`, `selected={} s`, `safety={}`, "
            "`rescalings={}`, `T0={} s`, `subdivisions={}`, "
            "`wheel_angle_bound={} <= limit={}`。".format(
                f(sel["required_duration_s"]), f(sel["selected_duration_s"]),
                f(sel["safety_factor"]), sel["rescalings"],
                f(sel["T0_s"]),
                sel["interval_subdivisions_per_segment"],
                f(sel["wheel_angle_bound_rad"]),
                f(sel["wheel_angle_limit_rad"])))
        add("")
        add("| metric | order | limit | upper@T0 | interval s | required (s) | witness vehicle/wheel/segment |")
        add("|---|---:|---:|---:|---|---:|---|")
        records = benchmark[name]["timing"]["selection"]["metrics_at_T0_s"]
        for metric in ("speed", "acceleration", "jerk", "yaw_rate",
                       "yaw_acceleration", "yaw_jerk",
                       "lateral_acceleration", "wheel_speed", "steer_rate",
                       "steer_acceleration"):
            record = records[metric]
            add("| {} | {} | {} | {} | [{}, {}] | {} | {}/{}/{} |".format(
                metric, record["order"], f(record["limit"]),
                f(record["upper_at_T0_s"]),
                f(record["upper_bound_interval"][0]),
                f(record["upper_bound_interval"][1]),
                f(record["required_duration_s"]), record["vehicle_id"],
                record["wheel_index"], record["segment_index"]))
        add("")
        add("绑定最不利记录：`quantity={}`, `order={}`, "
            "`vehicle_id={}`, `wheel_index={}`, `segment_index={}`, "
            "`interval=[{}, {}]`；recomputed candidate 与 "
            "`required_duration_s` 相等，`selected=required*1.01`。".format(
                sel["worst"]["quantity"], sel["worst"]["order"],
                sel["worst"]["vehicle_id"], sel["worst"]["wheel_index"],
                sel["worst"]["segment_index"], f(sel["worst"]["s0"]),
                f(sel["worst"]["s1"])))
        add("")

    add("## 5. 独立动态审核与全域证书")
    add("")
    add("几何前置（G1/C2）也必须通过：8 个运动实例的 geometry_audit 均 "
        "`passed=true`，刚性误差远低于 1e-6 m，C2 值/斜率/二阶误差均在 "
        "G1 容差内。")
    add("")
    add("| 实例 | geometry checks | 刚性误差 | 位置误差/容差 | C2 value/slope/second |")
    add("|---|---:|---:|---|---|")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        geometry = benchmark[name]["timing"]["geometry_audit"]
        sample_audit = geometry["metrics"]["sample_audit"]
        c2 = geometry["c2"]
        add("| {} | {} | {} | {}/{} | {}/{}/{} |".format(
            name, geometry["check_count"],
            f(sample_audit["max_pairwise_rigidity_error_m"]),
            f(sample_audit["max_position_error_m"]),
            f(sample_audit["position_tolerance_m"]),
            f(c2["max_value_error"]), f(c2["max_slope_error"]),
            f(c2["max_second_error"])))
    add("")
    add("运动实例从 T 和原始 profile/knots 重算：Hermite 段的 k/eta/u/g、"
        "D/Q/H 与轮系量使用 G1 区间外扩界，五次时间多项式使用区间 "
        "w/a/j；递归 q 细分至确认或预算失败。最大深度 20，最大叶数 "
        "200000；本运行 8 个运动实例均为 64 叶、证书通过，未出现 "
        "`BOUND_UNCONFIRMED`。")
    add("")
    add("端点和 hold 语义：起终速度、角速度和加速度为零；运动单侧端点 "
        "jerk 保留但不额外要求为零。默认 `FREE` 的终端 hold 保留路径"
        "实际 C、车头和理想轮角，不用 `omega/v` 在 v=0 时重算 C，也不"
        "把 C 清零；`STRAIGHT` 策略才要求 `kT=etaT=0`。内部节点 "
        "auto=right、终点 auto=left；所有内部节点 `w>0`，没有用曲率"
        "结点制造停车。")
    add("")
    add("| 实例 | audit checks | leaves | certificate | max util | max diff | max inverse | node continuity | node state | node jump |")
    add("|---|---:|---:|---|---:|---|---:|---:|---:|---:|")
    for case in cases:
        name = case["name"]
        audit = case["audit"]
        if name == "NO_MOTION_HOLD":
            add("| {} | {} | - | exact hold | - | - | - | - | {} | - |".format(
                name, audit["check_count"], f(audit["hold"]["max_state_error"])))
            continue
        diffs = audit["differential"]["max_errors"]
        max_diff = max(diffs.values())
        utilization = max(audit["max_utilization"].values())
        add("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            name, audit["check_count"], audit["leaves"],
            audit["whole_domain_certificate"], f(utilization),
            f(max_diff), f(audit["time_inverse"]["max_error_s"]),
            f(audit["nodes"]["max_continuity_error"]),
            f(audit["nodes"]["max_state_error"]),
            f(audit["nodes"]["max_jump_magnitude"])))
    add("")
    add("所有运动实例 `check_count=845`、`internal_node_count=7`、"
        "`leaves=64`、`whole_domain_certificate=true`。节点内部 "
        "`auto_region=motion`，左右区域都为 `motion`；终点 auto 左、"
        "内部 auto 右按任务约定审核。")
    add("")

    add("### 5.1 节点左右动态（全部实例）")
    add("")
    add("每条记录给出 `s,t,tau,w,jump_max,continuity,inverse`；左右 jerk "
        "向量为 3 车 × 2 分量，完整向量在同一实例的 "
        "`benchmark/results.json:<case> .dynamic_audit.metrics.nodes.records`。"
        "E1 的完整左右向量在 5.2 节。")
    add("")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        audit = case["audit"]
        add("#### {}".format(name))
        add("")
        add("| node | s | t | tau | w | jump_max | continuity | inverse | left/right region |")
        add("|---:|---:|---:|---:|---:|---:|---:|---:|---|")
        for index, record in enumerate(audit["nodes"]["records"], 1):
            add("| {} | {} | {} | {} | {} | {} | {} | {} | {}/{} |".format(
                index, f(record["s"]), f(record["t"]), f(record["tau"]),
                f(record["w"]), f(record["jump_max"]),
                f(record["max_continuity_error"]),
                f(record["inverse_error_s"]), record["left_region"],
                record["right_region"]))
        add("")
    add("### 5.2 E1 节点左右 jerk 向量")
    add("")
    add("```json")
    add(json.dumps({
        "E1_nodes": [
            {
                "node": index,
                "s": record["s"],
                "t": record["t"],
                "left_jerk": record["left_jerk"],
                "right_jerk": record["right_jerk"],
                "jump_max": record["jump_max"],
                "max_continuity_error": record["max_continuity_error"],
            }
            for index, record in enumerate(
                benchmark["E1"]["dynamic_audit"]["metrics"]["nodes"][
                    "records"], 1)
        ]
    }, indent=2, sort_keys=True))
    add("```")
    add("")

    add("## 6. 区间界与误差假设")
    add("")
    add("每个运动实例有 8 个 Hermite 段，每段 8 个证书叶；段级记录 "
        "`certified=true`，`max_depth=0`（本运行所有叶在初始区间即确认），"
        "每个段包含 10 个动态约束的 `upper_bound/limit`、q/tau 区间和 "
        "见证车辆。E1 的 8 段完整界如下（其它实例见 benchmark JSON）。")
    add("")
    add("| segment | s range | worst metric | worst bound/limit | certified | leaves |")
    add("|---:|---|---|---:|---|---:|")
    for index, row in enumerate(
            benchmark["E1"]["dynamic_audit"]["whole_segment_bounds"]):
        ratios = {
            metric: record["upper_bound"] / record["limit"]
            for metric, record in row["metrics"].items()}
        worst_metric = max(ratios, key=ratios.get)
        s_values = [record["s0"] for record in row["metrics"].values()] + [
            record["s1"] for record in row["metrics"].values()]
        add("| {} | [{}, {}] | {} | {} | {} | {} |".format(
            index, f(min(s_values)), f(max(s_values)), worst_metric,
            f(ratios[worst_metric]), row["certified"], row["leaves"]))
    add("")
    add("E1 每段每约束的完整上界：")
    add("")
    add("| segment | " + " | ".join(
        metric for metric in ("speed", "acceleration", "jerk", "yaw_rate",
                              "yaw_acceleration", "yaw_jerk",
                              "lateral_acceleration", "wheel_speed",
                              "steer_rate", "steer_acceleration")) + " |")
    add("|---:|" + "---:|" * 10)
    for index, row in enumerate(
            benchmark["E1"]["dynamic_audit"]["whole_segment_bounds"]):
        values = [f(row["metrics"][metric]["upper_bound"])
                  for metric in ("speed", "acceleration", "jerk", "yaw_rate",
                                 "yaw_acceleration", "yaw_jerk",
                                 "lateral_acceleration", "wheel_speed",
                                 "steer_rate", "steer_acceleration")]
        add("| {} | {} |".format(index, " | ".join(values)))
    add("")
    add("生产路径积分误差预算（G1 numerics v2）在动态审核的 "
        "`path_summary.integration_budget`：初始 Simpson panels 每个实例为 "
        "8×38；E1 的 `simpson_euclidean_m={}`，残差上界 "
        "`position_error_bound_m={}`，角度上界 "
        "`angle_error_bound_rad={}`。这些是路径几何积分预算，不是时间"
        "动力学证书的替代。"
        .format(f(benchmark["E1"]["dynamic_audit"]["path_summary"][
            "integration_budget"]["simpson_euclidean_m"]),
            f(benchmark["E1"]["dynamic_audit"]["path_summary"][
                "integration_budget"]["position_error_bound_m"]),
            f(benchmark["E1"]["dynamic_audit"]["path_summary"][
                "integration_budget"]["angle_error_bound_rad"])))
    add("")

    add("## 7. 独立差分审核")
    add("")
    add("oracle：独立更严格的 composite-Simpson 位置积分（`oracle_step_m="
        "0.002`，每段 376 个网格子区间）；生产速度/角速度、加速度/"
        "角加速度、jerk/角 jerk 与 5 点中心差分比较；同时报告截断和"
        "位置误差放大诊断。跨 g jump 或时间 hold 边界的 stencil 被跳过，"
        "边界使用解析左右极限。")
    add("")
    add("| 实例 | checked samples | velocity | acceleration | jerk | angular_rate | angular_accel | angular_jerk |")
    add("|---|---:|---:|---:|---:|---:|---:|---:|")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        errors = case["audit"]["differential"]["max_errors"]
        add("| {} | {} | {} | {} | {} | {} | {} | {} |".format(
            name, case["audit"]["differential"]["checked_sample_count"],
            f(errors["velocity"]), f(errors["acceleration"]),
            f(errors["jerk"]), f(errors["angular_rate"]),
            f(errors["angular_accel"]), f(errors["angular_jerk"])))
    add("")
    add("门槛：velocity/angular_rate `{}`，acceleration/angular_accel `{}`，"
        "jerk/angular_jerk `{}`。以下为全域最大：".format(
            f(aggregate["differential_tolerances"]["velocity"]),
            f(aggregate["differential_tolerances"]["acceleration"]),
            f(aggregate["differential_tolerances"]["jerk"])))
    add("")
    add("```json")
    add(json.dumps(aggregate["max_differential_errors"], indent=2,
                   sort_keys=True))
    add("```")
    add("")

    add("## 8. 100 个非均匀时间反解对照")
    add("")
    add("每个运动实例使用 100 个确定性非均匀 `tau=0.5-0.5*cos(pi*(i+0.5)/100)`；"
        "从原始 profile 重新求 `tau_for_fraction(s/S)`，与生产 "
        "`time_at_s(s)` 比较，并另外报告 source-tau 采样差。"
        "`max_error_s` 与 `tolerance=1e-9` 比较；"
        "`max_independent_error_s` 是 source-tau 诊断，不是 gate。"
        "完整样本在 `benchmark/results.json:<case> .dynamic_audit.metrics."
        "time_inverse.samples`；E1 的 100 个样本如下。")
    add("")
    add("| 实例 | samples | max_error (s) | max_source_error (s) | tolerance |")
    add("|---|---:|---:|---:|---:|")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        inverse = benchmark[name]["dynamic_audit"]["metrics"]["time_inverse"]
        add("| {} | {} | {} | {} | {} |".format(
            name, inverse["sample_count"], f(inverse["max_error_s"]),
            f(inverse["max_independent_error_s"]),
            f(inverse["tolerance_s"])))
    add("")
    add("### 8.1 E1 非均匀样本")
    add("")
    add("| i | tau | s | t (s) | source_t (s) | time_at_s (s) | error (s) | source_error (s) |")
    add("|---:|---:|---:|---:|---:|---:|---:|---:|")
    samples = benchmark["E1"]["dynamic_audit"]["metrics"]["time_inverse"][
        "samples"]
    for index, sample in enumerate(samples, 1):
        add("| {} | {} | {} | {} | {} | {} | {} | {} |".format(
            index, f(sample["tau"]), f(sample["s"]), f(sample["t"]),
            f(sample["source_t"]), f(sample["time_at_s"]),
            f(sample["error_s"]), f(sample["source_error_s"])))
    add("")

    add("## 9. 短时反例与篡改记录")
    add("")
    add("短时反例把 `T` 缩小后重新审核；所有运动实例均应被拒绝。记录的是"
        "真实超限，不是放宽或删除指标：")
    add("")
    add("| 实例 | short checks | 失败约束 | 值 | limit | 车辆 | 车轮 |")
    add("|---|---:|---|---:|---:|---:|---:|")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        short = case["short_duration"]
        failure = short["first_failure"]
        add("| {} | {} | {} | {} | {} | {} | {} |".format(
            name, short["check_count"], short["reason"],
            f(failure["value"]), f(failure["limit"]),
            failure["vehicle_id"], failure["wheel_index"]))
    add("")
    add("篡改模型序列化/时间字段后，所有运动实例的拒绝原因：")
    add("")
    add("| 实例 | `timing_T` | `extra_field` | `self_digest` |")
    add("|---|---|---|---|")
    for case in cases:
        name = case["name"]
        if name == "NO_MOTION_HOLD":
            continue
        tamper = case["tamper"]
        add("| {} | `{}` | `{}` | `{}` |".format(
            name, tamper["timing_T"], tamper["extra_field"],
            tamper["self_digest"]))
    add("")
    add("模型 JSON 不包含 `model_sha256`、audit 报告或自指摘要；`from_model_"
        "json` 只恢复结构/数学，不信任自报 passed。模型摘要由 benchmark "
        "从 `model_json` 字符串重新计算，并单独绑定请求摘要。")
    add("")

    add("## 10. 保护哈希与授权边界")
    add("")
    add("| 项目 | 值 |")
    add("|---|---|")
    add("| G1 protected_after SHA-256 | `{}` |".format(
        protection["g1_evidence"]["protected_after_sha256"]))
    add("| G1 protected tree | `{}` |".format(
        protection["g1_evidence"]["protected_tree_sha256"]))
    add("| G1 files | {} |".format(
        protection["g1_evidence"]["protected_file_count"]))
    add("| G2 accepted manifest SHA-256 | `{}` |".format(
        protection["g2_evidence"]["source_manifest_sha256"]))
    add("| G2 reconstructed protected tree | `{}` |".format(
        protection["g2_evidence"]["protected_tree_sha256"]))
    add("| G2 files | {} |".format(
        protection["g2_evidence"]["protected_file_count"]))
    add("| G3 current protected tree | `{}` |".format(
        protection["current"]["full_protected_tree_sha256"]))
    add("| G3 files | {} |".format(
        protection["current"]["current_file_count"]))
    add("| Comparison diff SHA-256 | `{}` |".format(
        hashlib.sha256(PROTECTION.read_bytes()).hexdigest()))
    add("| only_authorized_changes | {} |".format(
        protection["authorization"]["only_authorized_changes"]))
    add("")
    add("G3 仅有的 4 个新增受保护文件和 5 个授权修改：")
    add("")
    add("```text")
    for row in protection["added_vs_g2_accepted"]:
        add("A {}  {}".format(row["sha256"], row["path"]))
    for row in protection["changed_vs_g2_accepted"]:
        add("M {} -> {}  {}".format(row["g2_sha256"], row["g3_sha256"],
                                    row["path"]))
    add("```")
    add("")
    add("无删除；G2 accepted 运行文件逐个哈希复核。`docs/virtual_formation_a/"
        "CLAUDE_G3_TASK.md` 作为任务文档绑定在 30 个 source 文件中，"
        "不在受保护树根内，因此不是受保护新增项。")
    add("")

    add("## 11. 完整模型 JSON")
    add("")
    add("所有 9 个实例的完整 canonical `model_json` 位于 "
        "`benchmark/results.json` 的 `<case>.model_json`，并已在 "
        "`results.json` 绑定 `model_sha256` 和 `request_sha256`。摘要表：")
    add("")
    add("| 实例 | bytes | model SHA-256 | request SHA-256 |")
    add("|---|---:|---|---|")
    for case in cases:
        name = case["name"]
        add("| {} | {} | `{}` | `{}` |".format(
            name, len(benchmark[name]["model_json"]),
            case["model_sha256"], case["request_sha256"]))
    add("")
    add("### 11.1 E1 完整模型 JSON")
    add("")
    add("```json")
    add(benchmark["E1"]["model_json"])
    add("```")
    add("")
    add("### 11.2 NO_MOTION_HOLD 完整模型 JSON")
    add("")
    add("```json")
    add(benchmark["NO_MOTION_HOLD"]["model_json"])
    add("```")
    add("")

    add("## 12. 请求、环境与运行命令")
    add("")
    add("原始请求摘要绑定在每个模型核心的 `request_inputs`，并与 "
        "`request_sha256` 一致；E1 请求为 `FREE`，目标 "
        "`(5.982887884033, 0.299200999308, 0.2)`，起点 "
        "`(0,0,0)`，三车固定偏移 `(1,0),(0,1),(0,-1)` m。"
        "`NO_MOTION_HOLD` 为目标等于起点的显式 hold。")
    add("")
    add("证据运行环境记录在 `environment.json`：Python `{}`，"
        "CasADi `{}`，IPOPT `{}`，scipy/numpy 记录在 packages；"
        "git HEAD `{}`，branch `{}`。".format(
            environment["host"]["python_version"],
            environment["solver"]["casadi_version"],
            environment["solver"]["ipopt_plugin_available"],
            environment["git"]["head"], environment["git"]["branch"]))
    add("")
    add("精确命令和退出码见 `COMMANDS.md` 与 `logs/exit_codes.log`。"
        "三个 G3 suite 退出 0；benchmark 成功运行退出 0（两次开发尝试"
        "分别因选择表不一致和输出目录非空而退出 2，均在证据目录内，"
        "已披露）；未修改 G0 gate 退出 1（BLOCKED），披露 wrapper 的"
        "legacy regression 总体退出 1（FAIL，四个通过，"
        "`timing_continuous` 为 pre-existing `KeyError: dynamic_peaks`，"
        "已从 pristine HEAD 复现）。")
    add("")
    add("| 实际命令 | 退出码 |")
    add("|---|---:|")
    add("| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_timing.py' -v` | 0 |")
    add("| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v` | 0 |")
    add("| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v` | 0 |")
    add("| `python3 -B tools/virtual_a_benchmark.py --stage G3 --output \"$VA_RUN/benchmark\"` | 0 |")
    add("| `python3 -B tools/virtual_a_baseline.py --output \"$VA_RUN/legacy\" --regression timing` | 1 (BLOCKED) |")
    add("| `python3 -B \"$VA_RUN/baseline_runner_rework_docs_and_fixtures.py\" --output \"$VA_RUN/legacy\" --regression timing` | 1 (FAIL) |")
    add("| pristine-HEAD `test_continuous_retiming.py` probe | 1 (pre-existing failure) |")
    add("| `python3 -B \"$VA_RUN/analysis/protection_diff_vs_g2_accepted.py\"` | 0 |")
    add("| `python3 -B \"$VA_RUN/analysis/assemble_evidence.py\"` | 0 |")
    add("| `python3 -B \"$VA_RUN/analysis/write_environment.py\"` | 0 |")
    add("")
    add("manifest 生成和独立验证在报告写入后执行；它们的状态只由 "
        "`logs/source_manifest_generation.log`、"
        "`logs/source_manifest_verification.log` 和最终 manifest 报告，"
        "不在本报告预先声称 PASS。")
    add("")

    add("## 13. Legacy 真实性")
    add("")
    add("Legacy 结果不是 PASS：")
    add("")
    add("| case | status |")
    add("|---|---|")
    for name, record in sorted(results["legacy"]["disclosed_wrapper"][
            "cases"].items()):
        add("| {} | {} |".format(name, record["status"]))
    add("")
    add("未修改 gate 记录 `UNKNOWN_WORKSPACE_MODIFICATIONS`，"
        "未修改测试或放宽限值。pristine HEAD 探针在 "
        "`tests/test_continuous_retiming.py:271` 复现 "
        "`KeyError: 'dynamic_peaks'`，证明是基线陈旧而非 G3 造成。")
    add("")

    add("## 14. 证据文件与验证")
    add("")
    add("- `benchmark/summary.json`：151 checks，0 failed。")
    add("- `benchmark/results.json`：9 个完整模型、timing、动态审核、"
        "短时反例、篡改和差分/反解样本。")
    add("- `results.json`：聚焦结果、真实 legacy 状态、保护摘要和非声明。")
    add("- `analysis/protection_diff_vs_g2_accepted.json`：G1/G2/G3 "
        "保护树和精确授权差异。")
    add("- `analysis/write_source_manifest.py` / `analysis/verify_source_"
        "manifest.py`：30 个 source 文件和全部 run 文件的 manifest 与"
        "独立重算验证。")
    add("- `source_manifest.json`、`logs/source_manifest_generation.log`、"
        "`logs/source_manifest_verification.log`：最终哈希绑定与独立验证；"
        "这三个文件自身不进入 manifest 的 run_files 以避免自指。")
    add("")

    add("## 15. 停止条件")
    add("")
    add("本报告提交后停止：等待 Codex 对 G3 的 ACCEPTED 和用户下达 G4。"
        "不进入 G4，不做地图、ROS、Gazebo、时间最优、碰撞或可执行性工作；"
        "不提交 Git，不创建分支，不操作其它 worktree。")
    add("")

    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(OUT.relative_to(REPO_ROOT)),
        "bytes": OUT.stat().st_size,
        "lines": len(lines),
        "cases": len(cases),
        "benchmark_checks": len(summary["checks"]),
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()