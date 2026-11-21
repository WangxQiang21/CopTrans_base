#!/usr/bin/env bash
set -euo pipefail

demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
overlay_dir="${demo_dir}/catkin_ws"
map_root="${demo_dir}/maps"
map_index="${map_root}/map_index.tsv"

# This checkout is the continuous-mode-path algorithm version.
branch_backend="mode_path_v2"
branch_name="continuous-mode-path"
demo_kind="formation"
for option in "$@"; do
  [[ "$option" == --single ]] && demo_kind="single"
done

usage() {
  if [[ "$demo_kind" == formation ]]; then
    cat <<'EOF'
用法：./run_demo.sh [--map map01] [--scale 比例] [--backend uniform_v02|virtual_rigid_a|legacy] [auto_goal:=false] [rviz:=true]
默认启动三车楔形编队：car1(1,0)、car2(0,1)、car3(0,-1)，单位米。
2D Pose Estimate 设置编队参考点 F 的起点与航向，2D Nav Goal 设置 F 的终点与航向。
--collect-cases 采集编队回归用例，保存到 work_flow/test_scenarios/cases。
采集时每次指定终点即保存，用 --cases-dir 指定其他用例目录。
车身默认以实心长方体显示；show_footprints:=true 可叠加足迹轮廓。
编队后端默认 virtual_rigid_a（G5 虚拟刚性 A 研究入口，guidance=corridor、
enforcement=hard、auto_goal=false；同一原子 /formation/playback_plan 信封，需先做
2D Pose Estimate 明确新实验，virtual_a_goal_policy:=free|straight 选择终点策略）；
virtual_a_guidance_mode:=corridor|legacy_route|footprint_seed 与
virtual_a_corridor_enforcement:=hard|soft 可显式覆盖默认研究入口；
virtual_a_formation_corridor_scope:=f_only|three_vehicle_union_v1 显式选择走廊
范围（默认 f_only，保持历史 F-only gate；union 为 TASK03 显式实验入口）；
backend:=uniform_v02 启动 V0.2 自主统一规划；uniform_direct_goal_translation:=true
启用可审核的 A/O 连接（含固定航向跨走廊）及 S 对齐、A 枢轴连接。
uniform_global_translation_route:=false 可关闭固定航向跨走廊候选用于对照。
uniform_heading_aligned_route:=false 将全局栅格搜索限制在地图坐标轴，用于对照斜向通行。
uniform_heading_transition_route:=false 关闭跨编队航向的转弯连接。
uniform_continuous_a:=true 启用统一时钟中的变曲率 A 局部细化。
uniform_goal_formation_heading_free:=false 可要求编队停靠时匹配目标箭头朝向。
--backend legacy 或
backend:=legacy 仅启动旧 C++ 前端与旧优化器用于对照。
同一时刻只会启动一个规划发布后端，不会同时发布两套轨迹。
编队默认比例见 maps/map_index.tsv，可用 --scale 指定绝对比例。
--list-maps 查看地图。--single 使用原单车流程（含 --collect-data）。
EOF
    return
  fi
  cat <<'EOF'
用法：
  ./run_demo.sh [ROS参数...]
  ./run_demo.sh --map map01/map001、完整目录名、旧名称或地图目录 [ROS参数...]
  ./run_demo.sh --map map01/map001、完整目录名、旧名称或地图目录 [--map-scale|--scale|-scale 比例] [ROS参数...]
  ./run_demo.sh --list-maps
  ./run_demo.sh --map map04 --collect-data [--data-root 仓库外目录]

示例：
  ./run_demo.sh --map P12_env1
  ./run_demo.sh --map map01
  ./run_demo.sh --map - map01
  ./run_demo.sh --map map04-p12-maze-env1
  ./run_demo.sh --map /path/to/map_directory rviz:=true
  ./run_demo.sh --map P12_env1 --map-scale 0.85
  ./run_demo.sh --map map01 -scale 0.7
  ./run_demo.sh --map P12_env2 target_v:=2.0
  ./run_demo.sh --map S09_underground_garage
  ./run_demo.sh --map map015
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2 direction_policy:=forward_only
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2 allow_reverse_fallback:=false

map01 默认按 0.75、map03 按 0.7、map04 按 0.5、map05 按 0.7、
map07 按 0.8、map08 按 1.2、map15 按 0.8 比例运行；map02、map06、
map09 和 map16 默认按 1.0。
用 --map-scale、--scale 或 -scale 覆盖，比例以地图中心为基准，必须大于 0。
地图栅格不会被重采样。map08 和 map09 的锯齿已做栅格平滑处理。

本分支固定使用 planning_backend:=mode_path_v2（连续模式路径后端）。
同模式控制节点连续运行，切模式固定静止 0.3 秒，并复核连续终点和扫掠。
省略后端参数时自动使用本分支后端；指定其他后端会报错，请用 git switch 切换分支。
目标速度默认 2.0 m/s，可用 target_v:=数值 覆盖；弯道不因曲率主动减速。
起终点和切模式处保留加减速；短路段可能来不及达到目标速度。
默认在严格认证通过的候选中选择执行时间最短的路线，净空/平顺性作近乎等时的次级比较。
受限动态对照可用 config_file:=绝对路径/params_restricted.yaml 启动。
倒车优先留到后段，极端情况允许提前；同模式连续运行，切模式停车重配置。
direction_policy:=bidirectional（新后端默认）允许正反向候选按共同代价竞争；
forward_only 用于只允许正向的对照。旧参数 allow_reverse_fallback:=true/false
仍可用并会映射到这两个策略；同时指定新旧参数且相互冲突会报配置错误。

采集模式：打开本地标注网页，在 RViz 手动设置起终点，结果出来后选优质/参考/剔除。
采集默认关闭自动终点；可显式传 auto_goal:=true 使用预设任务。
数据自动保存到仓库外 expv3_experiment_archive/teacher_collection。
二次筛选：python3 tools/review_collected_data.py --reanalyze

手动选图时，如果地图目录含 scene_meta.yaml，默认使用其中的预设起点和终点，
并使用空场景标注；可传 auto_goal:=false 关闭自动终点，再在 RViz 用
2D Pose Estimate 设置起点、用 2D Nav Goal 设置终点。
EOF
}

collect_data=false
data_root=""
collect_cases=false
cases_dir=""
requested_map=""
requested_map_scale=""
requested_backend=""
list_maps=false
dry_run=false
has_map_dir_arg=false
launch_args=()
while (($#)); do
  case "$1" in
    --dry-run)
      dry_run=true
      shift
      ;;
    --single)
      shift
      ;;
    --collect-data)
      collect_data=true
      shift
      ;;
    --collect-cases)
      collect_cases=true
      shift
      ;;
    --cases-dir)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "错误：--cases-dir 需要保存目录。" >&2
        exit 2
      fi
      cases_dir="$2"
      shift 2
      ;;
    --data-root)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "错误：--data-root 需要保存目录。" >&2
        exit 2
      fi
      data_root="$2"
      shift 2
      ;;
    --map|-m)
      if [[ -n "${requested_map}" ]]; then
        echo "错误：--map 只能指定一次；设置地图比例请写 --scale 0.7 或 -scale 0.7（不要重复写 --map）。" >&2
        usage >&2
        exit 2
      fi
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "错误：--map 需要且只能指定一个地图名或目录。" >&2
        usage >&2
        exit 2
      fi
      if [[ "$2" == "-" ]]; then
        if (($# < 3)) || [[ -z "$3" ]]; then
          echo "错误：--map - 后需要地图编号或名称。" >&2
          usage >&2
          exit 2
        fi
        requested_map="$3"
        shift 3
      else
        requested_map="$2"
        shift 2
      fi
      ;;
    --map=*)
      if [[ -n "${requested_map}" ]]; then
        echo "错误：--map 只能指定一次。" >&2
        exit 2
      fi
      requested_map="${1#--map=}"
      if [[ -z "${requested_map}" ]]; then
        echo "错误：--map 后需要地图名或地图目录。" >&2
        exit 2
      fi
      shift
      ;;
    --map-scale|--scale|-scale)
      if (($# < 2)) || [[ -z "$2" ]] || [[ -n "${requested_map_scale}" ]]; then
        echo "错误：地图缩放参数需要且只能指定一个比例。" >&2
        usage >&2
        exit 2
      fi
      requested_map_scale="$2"
      shift 2
      ;;
    --map-scale=*|--scale=*|-scale=*)
      if [[ -n "${requested_map_scale}" ]]; then
        echo "错误：地图缩放参数只能指定一次。" >&2
        exit 2
      fi
      requested_map_scale="${1#*=}"
      if [[ -z "${requested_map_scale}" ]]; then
        echo "错误：地图缩放参数后需要比例。" >&2
        exit 2
      fi
      shift
      ;;
    --backend)
      if (($# < 2)) || [[ -z "$2" ]]; then
        echo "错误：--backend 需要 uniform_v02、virtual_rigid_a 或 legacy。" >&2
        exit 2
      fi
      if [[ -n "${requested_backend}" && "${requested_backend}" != "$2" ]]; then
        echo "错误：编队后端只能指定一次。" >&2
        exit 2
      fi
      requested_backend="$2"
      shift 2
      ;;
    --backend=*)
      if [[ -n "${requested_backend}" && "${requested_backend}" != "${1#--backend=}" ]]; then
        echo "错误：编队后端只能指定一次。" >&2
        exit 2
      fi
      requested_backend="${1#--backend=}"
      if [[ -z "${requested_backend}" ]]; then
        echo "错误：--backend 后需要 uniform_v02、virtual_rigid_a 或 legacy。" >&2
        exit 2
      fi
      shift
      ;;
    --list-maps)
      list_maps=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      [[ "$1" == map_dir:=* ]] && has_map_dir_arg=true
      launch_args+=("$1")
      shift
      ;;
  esac
done

if [[ "$demo_kind" == formation && "$collect_data" == true ]]; then
  echo "错误：现有 --collect-data 采集器仅支持单车，请加 --single。编队规划诊断见 /formation/quality_report。" >&2
  exit 2
fi
if [[ "$collect_cases" == true && "$demo_kind" != formation ]] ||
   [[ -n "$cases_dir" && "$collect_cases" != true ]]; then
  echo "错误：--collect-cases 只用于编队。--cases-dir 需与它一起使用。" >&2
  exit 2
fi
if [[ "$demo_kind" == single && -n "${requested_backend}" ]]; then
  echo "错误：--backend 仅用于编队 demo；单车流程固定使用 ${branch_backend}。" >&2
  exit 2
fi
if [[ "$demo_kind" == formation && -z "$requested_map" && "$has_map_dir_arg" == false ]]; then
  requested_map="map16"
fi

if [[ -n "${data_root}" && "${collect_data}" != true ]]; then
  echo "错误：--data-root 需要与 --collect-data 一起使用。" >&2
  exit 2
fi

if [[ "${list_maps}" == true ]]; then
  echo "地图编号（目录名；括号内依次为三位编号写法和兼容旧名称）："
  while IFS=$'\t' read -r map_folder legacy_name category formation_scale; do
    [[ -z "${map_folder}" || "${map_folder}" == \#* ]] && continue
    [[ -f "${map_root}/${map_folder}/map.yaml" ]] || continue
    if [[ "${map_folder}" =~ ^map([0-9]{2})- ]]; then
      printf '  %-38s (map0%s, %s)\n' "${map_folder}" "${BASH_REMATCH[1]}" "${legacy_name}"
    else
      printf '  %-38s (%s)\n' "${map_folder}" "${legacy_name}"
    fi
  done < "${map_index}"
  exit 0
fi

manual_map=false
selected_map_dir=""
if [[ -n "${requested_map}" ]]; then
  if [[ "${has_map_dir_arg}" == true ]]; then
    echo "错误：--map 与 map_dir:= 不能同时指定。" >&2
    exit 2
  fi
  manual_map=true
  map_lookup_name="${requested_map}"
  if [[ "${requested_map}" =~ ^map0([0-9]{2})$ ]]; then
    map_lookup_name="map${BASH_REMATCH[1]}"
  fi
  if [[ -d "${requested_map}" ]]; then
    selected_map_dir="$(cd -- "${requested_map}" && pwd)"
  elif [[ -d "${map_root}/${requested_map}" ]]; then
    selected_map_dir="${map_root}/${requested_map}"
  else
    while IFS=$'\t' read -r map_folder legacy_name category formation_scale; do
      [[ -z "${map_folder}" || "${map_folder}" == \#* ]] && continue
      if [[ "${requested_map}" == "${legacy_name}" ]] || \
         [[ "${map_lookup_name}" =~ ^map[0-9]{2}$ && "${map_folder}" == "${map_lookup_name}-"* ]]; then
        selected_map_dir="${map_root}/${map_folder}"
        break
      fi
    done < "${map_index}"
    if [[ -z "${selected_map_dir}" ]]; then
      echo "错误：找不到地图名或目录：${requested_map}" >&2
      echo "地图列表：./run_demo.sh --list-maps" >&2
      exit 2
    fi
  fi
  if [[ ! -f "${selected_map_dir}/map.yaml" ]]; then
    echo "错误：地图目录缺少 map.yaml：${selected_map_dir}" >&2
    exit 2
  fi
  launch_args+=("map_dir:=${selected_map_dir}")
fi

# Also recognize a map_dir:= argument so runtime scaling works with either
# the --map convenience option or ordinary roslaunch arguments.
for arg in "${launch_args[@]}"; do
  if [[ "${arg}" == map_dir:=* ]]; then
    selected_map_dir="${arg#map_dir:=}"
    has_map_dir_arg=true
    break
  fi
done

if [[ "$demo_kind" == formation && -z "$selected_map_dir" ]]; then
  selected_map_dir="$map_root/map16-s09-underground-garage"
  launch_args+=("map_dir:=$selected_map_dir")
fi

source_map_dir=""
if [[ "$collect_cases" == true ]]; then
  source_map_dir="$(realpath -- "$selected_map_dir")"
fi
runtime_map_parent=""
cleanup_runtime_map() {
  if [[ -n "${runtime_map_parent}" ]]; then
    rm -rf -- "${runtime_map_parent}"
  fi
}
trap cleanup_runtime_map EXIT

effective_map_scale="${requested_map_scale:-1.0}"
if [[ -z "$requested_map_scale" && -n "$selected_map_dir" ]]; then
  selected_folder="$(basename -- "${selected_map_dir%/}")"
  if [[ "$demo_kind" == formation ]]; then
    effective_map_scale="3.75"
    while IFS=$'\t' read -r map_folder legacy_name category formation_scale; do
      if [[ "$map_folder" == "$selected_folder" ]]; then
        effective_map_scale="$formation_scale"
        break
      fi
    done < "$map_index"
  else
    case "$selected_folder" in
    map01-p11-narrow-case-a) effective_map_scale="0.75" ;;
    map03-p11-narrow-case-c) effective_map_scale="0.7" ;;
    map04-p12-maze-env1) effective_map_scale="0.5" ;;
    map05-p12-parking-row-env2) effective_map_scale="0.7" ;;
    map07-p12-obstacle-field-env4) effective_map_scale="0.8" ;;
    map08-p16-complex-parking) effective_map_scale="1.2" ;;
    map15-n06-two-stage-offset-passage) effective_map_scale="0.8" ;;
    esac
  fi
fi
if [[ -n "${requested_map_scale}" || "${effective_map_scale}" != "1.0" ]]; then
  if [[ -z "${selected_map_dir}" || ! -d "${selected_map_dir}" ]]; then
    echo "错误：--map-scale 需要有效的 --map 或 map_dir:= 地图目录。" >&2
    exit 2
  fi
  selected_map_dir="$(cd -- "${selected_map_dir}" && pwd)"
  if ! python3 -c 'import math,sys; x=float(sys.argv[1]); sys.exit(0 if math.isfinite(x) and x>0 else 1)' "${effective_map_scale}"; then
    echo "错误：地图比例必须是大于 0 的有限数值：${effective_map_scale}" >&2
    exit 2
  fi
  runtime_map_parent="$(mktemp -d "${TMPDIR:-/tmp}/expv3-map-scale.XXXXXX")"
  runtime_map_dir="${runtime_map_parent}/$(basename -- "${selected_map_dir}")"
  python3 "${demo_dir}/tools/scale_map_runtime.py" \
    "${selected_map_dir}" "${runtime_map_dir}" "${effective_map_scale}"
  replaced_map_dir=false
  scaled_launch_args=()
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == map_dir:=* ]]; then
      scaled_launch_args+=("map_dir:=${runtime_map_dir}")
      replaced_map_dir=true
    else
      scaled_launch_args+=("${arg}")
    fi
  done
  launch_args=("${scaled_launch_args[@]}")
  if [[ "${replaced_map_dir}" == false ]]; then
    launch_args+=("map_dir:=${runtime_map_dir}")
  fi
  manual_map=true
  echo "[expv3] 地图比例 ${effective_map_scale}，中心点固定；栅格尺寸保持不变。"
fi

# Keep each algorithm branch tied to its own backend. Reject other backends so
# changing algorithm versions requires switching the Git branch.
has_planning_backend=false
if [[ "$demo_kind" == single ]]; then
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == planning_backend:=* ]]; then
      requested_backend="${arg#planning_backend:=}"
      if [[ "${requested_backend}" != "${branch_backend}" ]]; then
        echo "错误：当前分支 ${branch_name} 固定使用 ${branch_backend}，拒绝 ${requested_backend}。请用 git switch 切换算法分支。" >&2
        exit 2
      fi
      has_planning_backend=true
    fi
  done
  if [[ "${has_planning_backend}" == false ]]; then
    launch_args+=("planning_backend:=${branch_backend}")
  fi
fi

# FR05: the formation demo runs exactly one trajectory-publishing backend.
# --backend is the CLI convenience form; backend:= in the pass-through
# arguments is the roslaunch form.  Both are accepted, but they may not
# disagree, and only the known backends are allowed.
if [[ "$demo_kind" == formation ]]; then
  formation_backend=""
  backend_in_launch_args=false
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == backend:=* ]]; then
      candidate="${arg#backend:=}"
      if [[ "${backend_in_launch_args}" == true && "${candidate}" != "${formation_backend}" ]]; then
        echo "错误：backend:= 只能指定一次。" >&2
        exit 2
      fi
      formation_backend="${candidate}"
      backend_in_launch_args=true
    fi
  done
  if [[ -n "${requested_backend}" ]]; then
    if [[ "${backend_in_launch_args}" == true && "${formation_backend}" != "${requested_backend}" ]]; then
      echo "错误：--backend ${requested_backend} 与 backend:=${formation_backend} 冲突。" >&2
      exit 2
    fi
    formation_backend="${requested_backend}"
  fi
  if [[ -z "${formation_backend}" ]]; then
    formation_backend="virtual_rigid_a"
    [[ "$collect_cases" != true ]] || formation_backend="uniform_v02"
  fi
  if [[ "${formation_backend}" != "uniform_v02" && "${formation_backend}" != "virtual_rigid_a" && "${formation_backend}" != "legacy" ]]; then
    echo "错误：formation backend 只能是 uniform_v02、virtual_rigid_a 或 legacy，收到 ${formation_backend}。" >&2
    exit 2
  fi
  if [[ "${backend_in_launch_args}" == false ]]; then
    launch_args+=("backend:=${formation_backend}")
  fi
  # Research entry: virtual_rigid_a defaults to corridor guidance with hard
  # enforcement and a manual start pose.  Only missing values are injected;
  # an explicitly repeated parameter name (even with the same value) exits 2.
  # Explicit uniform_v02/legacy keep their historical auto_goal behaviour.
  if [[ "${formation_backend}" == "virtual_rigid_a" ]]; then
    guidance_seen=0
    enforcement_seen=0
    formation_scope_seen=0
    virtual_auto_goal_seen=0
    for arg in "${launch_args[@]}"; do
      case "${arg}" in
        virtual_a_guidance_mode:=*)
          guidance_seen=$((guidance_seen + 1))
          ;;
        virtual_a_corridor_enforcement:=*)
          enforcement_seen=$((enforcement_seen + 1))
          ;;
        virtual_a_formation_corridor_scope:=*)
          formation_scope_seen=$((formation_scope_seen + 1))
          ;;
        auto_goal:=*)
          virtual_auto_goal_seen=$((virtual_auto_goal_seen + 1))
          ;;
      esac
    done
    if (( guidance_seen > 1 )); then
      echo "错误：virtual_a_guidance_mode:= 只能指定一次。" >&2
      exit 2
    fi
    if (( enforcement_seen > 1 )); then
      echo "错误：virtual_a_corridor_enforcement:= 只能指定一次。" >&2
      exit 2
    fi
    if (( formation_scope_seen > 1 )); then
      echo "错误：virtual_a_formation_corridor_scope:= 只能指定一次。" >&2
      exit 2
    fi
    if (( virtual_auto_goal_seen > 1 )); then
      echo "错误：auto_goal:= 只能指定一次。" >&2
      exit 2
    fi
    (( guidance_seen == 1 )) || launch_args+=("virtual_a_guidance_mode:=corridor")
    (( enforcement_seen == 1 )) || launch_args+=("virtual_a_corridor_enforcement:=hard")
    # TASK03: f_only stays the default until the union delivery is accepted.
    (( formation_scope_seen == 1 )) || launch_args+=("virtual_a_formation_corridor_scope:=f_only")
    (( virtual_auto_goal_seen == 1 )) || launch_args+=("auto_goal:=false")
  fi
fi

if [[ "$demo_kind" == formation && "$formation_backend" == uniform_v02 ]]; then
  goal_heading_policy_seen=false
  for arg in "${launch_args[@]}"; do
    [[ "$arg" == uniform_goal_formation_heading_free:=* ]] && goal_heading_policy_seen=true
  done
  [[ "$goal_heading_policy_seen" == true ]] || launch_args+=("uniform_goal_formation_heading_free:=true")
fi

if [[ "$collect_cases" == true ]]; then
  if [[ "$formation_backend" == legacy ]]; then
    echo "错误：编队采集支持 uniform_v02 和 virtual_rigid_a。" >&2
    exit 2
  fi
  collection_args=()
  for arg in "${launch_args[@]}"; do
    case "$arg" in
      auto_goal:=false) continue ;;
      planner:=true|goal_bridge:=true|start_map:=true) ;;
      auto_goal:=*|planner:=*|goal_bridge:=*|start_map:=*|case_collection_*)
        echo "错误：参数 $arg 与编队采集模式冲突。" >&2
        exit 2
        ;;
    esac
    collection_args+=("$arg")
  done
  launch_args=("${collection_args[@]}" "auto_goal:=false")
  if [[ "$formation_backend" == uniform_v02 ]]; then
    direct_seen=false
    for arg in "${launch_args[@]}"; do
      [[ "$arg" == uniform_direct_goal_translation:=* ]] && direct_seen=true
    done
    [[ "$direct_seen" == true ]] || launch_args+=("uniform_direct_goal_translation:=true")
  fi
fi

if [[ "${collect_data}" == true ]]; then
  launch_args+=("collect_data:=true" "map_scale:=${effective_map_scale}" "project_root:=${demo_dir}")
  if [[ -z "${data_root}" ]]; then
    data_root="$(dirname -- "${demo_dir}")/expv3_experiment_archive/teacher_collection"
  fi
  launch_args+=("data_root:=${data_root}")
  collection_has_auto_goal=false
  for arg in "${launch_args[@]}"; do
    [[ "${arg}" == auto_goal:=* ]] && collection_has_auto_goal=true
  done
  [[ "${collection_has_auto_goal}" == true ]] || launch_args+=("auto_goal:=false")
  echo "[expv3] 人工采集模式，数据目录：${data_root}"
fi

has_auto_goal=false
has_scene=false
has_rviz=false
for arg in "${launch_args[@]}"; do
  [[ "${arg}" == auto_goal:=* ]] && has_auto_goal=true
  [[ "${arg}" == scene:=* ]] && has_scene=true
  [[ "${arg}" == rviz:=* ]] && has_rviz=true
done
if [[ "${manual_map}" == false && "${has_map_dir_arg}" == true ]]; then
  manual_map=true
fi
if [[ "${manual_map}" == true ]]; then
  [[ "${has_scene}" == true ]] || launch_args+=("scene:=manual_map")
  if [[ "${has_auto_goal}" == false ]]; then
    if [[ -n "${selected_map_dir}" && -f "${selected_map_dir}/scene_meta.yaml" ]]; then
      launch_args+=("auto_goal:=true")
    else
      launch_args+=("auto_goal:=false")
    fi
  fi
elif [[ "${has_auto_goal}" == false ]]; then
  launch_args+=("auto_goal:=true")
fi
if [[ "${has_rviz}" == false && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[expv3] 未检测到图形桌面，自动使用 rviz:=false。"
  launch_args+=("rviz:=false")
fi

if [[ "$demo_kind" == single ]]; then
  python3 "${demo_dir}/tools/describe_run_context.py" \
    "${demo_dir}" "${selected_map_dir:-}" "${effective_map_scale}" "${launch_args[@]}"
  demo_launch="demo.launch"
else
  demo_launch="formation_demo.launch"
  view_map_dir="${runtime_map_dir:-$selected_map_dir}"
  python3 "${demo_dir}/tools/describe_run_context.py" --formation \
    "$view_map_dir" "$effective_map_scale" "${launch_args[@]}"
  has_rviz_config=false
  for arg in "${launch_args[@]}"; do
    [[ "$arg" == rviz_config:=* ]] && has_rviz_config=true
  done
  if [[ "$has_rviz_config" == false ]]; then
    if [[ -z "$runtime_map_parent" ]]; then
      runtime_map_parent="$(mktemp -d "${TMPDIR:-/tmp}/expv3-formation-view.XXXXXX")"
    fi
    view_options=()
    [[ "$collect_cases" != true ]] || view_options+=("--collect-cases")
    python3 "${demo_dir}/tools/configure_formation_view.py" "$view_map_dir" \
      "${overlay_dir}/src/fwis_v3_demo/rviz/formation_demo.rviz" \
      "${runtime_map_parent}/formation.rviz" "${view_options[@]}"
    launch_args+=("rviz_config:=${runtime_map_parent}/formation.rviz")
  fi
  [[ "$has_rviz" == true || -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || launch_args+=("rviz:=true")
  if [[ "$collect_cases" == true ]]; then
    if [[ -z "$runtime_map_parent" ]]; then
      runtime_map_parent="$(mktemp -d "${TMPDIR:-/tmp}/expv3-scenario.XXXXXX")"
    fi
    profile="${runtime_map_parent}/scenario_profile.json"
    python3 "${demo_dir}/tools/run_formation_cases.py" _profile "$profile" \
      "$source_map_dir" "$effective_map_scale" "${launch_args[@]}"
    cases_dir="$(realpath -m -- "${cases_dir:-${demo_dir}/work_flow/test_scenarios/cases}")"
    launch_args+=("case_collection_dir:=${cases_dir}" "case_collection_profile:=${profile}")
    echo "[expv3] 编队用例采集：先用 2D Pose Estimate 设置起点，再用 2D Nav Goal 保存用例。"
    echo "[expv3] 用例目录：${cases_dir}"
  fi
fi

if [[ "$dry_run" == true ]]; then
  printf 'roslaunch fwis_v3_demo %s' "$demo_launch"
  printf ' %q' "${launch_args[@]}"
  printf '\n'
  exit 0
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "错误：未找到 ROS Noetic (/opt/ros/noetic/setup.bash)。" >&2
  exit 1
fi
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/noetic/setup.bash
set -u

echo "[expv3] 构建 ROS overlay..."
catkin_make -C "${overlay_dir}" -j2 >/dev/null
set +u
source "${overlay_dir}/devel/setup.bash"
set -u
bonminseed_plugin_dir="${overlay_dir}/devel/lib/fwis_v3_optimizer/casadi_plugins"
if [[ -f "${bonminseed_plugin_dir}/libcasadi_nlpsol_bonminseed.so" ]]; then
  export CASADIPATH="${bonminseed_plugin_dir}${CASADIPATH:+:${CASADIPATH}}"
fi

echo "[expv3] 启动 Demo：${launch_args[*]}"
roslaunch fwis_v3_demo "$demo_launch" "${launch_args[@]}"
