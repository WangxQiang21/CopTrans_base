#!/usr/bin/env bash
set -euo pipefail

demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
overlay_dir="${demo_dir}/catkin_ws"
map_root="${demo_dir}/maps"
map_index="${map_root}/map_index.tsv"

usage() {
  cat <<'EOF'
用法：
  ./run_demo.sh [ROS参数...]
  ./run_demo.sh --map map01/map001、完整目录名、旧名称或地图目录 [ROS参数...]
  ./run_demo.sh --map map01/map001、完整目录名、旧名称或地图目录 [--map-scale|--scale|-scale 比例] [ROS参数...]
  ./run_demo.sh --list-maps

示例：
  ./run_demo.sh --map P12_env1
  ./run_demo.sh --map map01
  ./run_demo.sh --map - map01
  ./run_demo.sh --map map04-p12-maze-env1
  ./run_demo.sh --map /path/to/map_directory rviz:=true
  ./run_demo.sh --map P12_env1 --map-scale 0.85
  ./run_demo.sh --map map01 -scale 0.7
  ./run_demo.sh --map P12_env2 target_v:=2.0
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v1
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2
  ./run_demo.sh --map P12_env1 planning_backend:=legacy
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2 direction_policy:=forward_only
  ./run_demo.sh --map P12_env1 planning_backend:=mode_path_v2 allow_reverse_fallback:=false
  ./run_demo.sh --map S09_underground_garage
  ./run_demo.sh --map map015

map01 默认按 0.75、map03 按 0.7、map04 按 0.5、map05 按 0.7、
map07 按 0.8、map08 按 1.2、map15 按 0.8 比例运行；map02、map06、
map09 和 map16 默认按 1.0。
用 --map-scale、--scale 或 -scale 覆盖，比例以地图中心为基准，必须大于 0。
地图栅格不会被重采样。map08 和 map09 的锯齿已做栅格平滑处理。

planning_backend:=mode_path_v1 使用 R02 恒定原语后端；
planning_backend:=mode_path_v2 使用 R03 连续控制/连续时间原型，
同模式控制节点连续运行，切模式固定静止 0.3 秒，并复核连续终点和扫掠。
省略 planning_backend 时默认使用 mode_path_v2；它仍在跨地图验证中。
目标速度默认 2.0 m/s，可用 target_v:=数值 覆盖；弯道不因曲率主动减速。
起终点和切模式处保留加减速；短路段可能来不及达到目标速度。
默认在严格认证通过的候选中选择执行时间最短的路线，净空/平顺性作近乎等时的次级比较。
受限动态对照可用 config_file:=绝对路径/params_restricted.yaml 启动。
倒车优先留到后段，极端情况允许提前；同模式连续运行，切模式停车重配置。
planning_backend:=legacy 使用旧后端作对照；显式设置会覆盖默认值。
direction_policy:=bidirectional（新后端默认）允许正反向候选按共同代价竞争；
forward_only 用于只允许正向的对照。旧参数 allow_reverse_fallback:=true/false
仍可用并会映射到这两个策略；同时指定新旧参数且相互冲突会报配置错误。

手动选图时，如果地图目录含 scene_meta.yaml，默认使用其中的预设起点和终点，
并使用空场景标注；可传 auto_goal:=false 关闭自动终点，再在 RViz 用
2D Pose Estimate 设置起点、用 2D Nav Goal 设置终点。
EOF
}

requested_map=""
requested_map_scale=""
list_maps=false
has_map_dir_arg=false
launch_args=()
while (($#)); do
  case "$1" in
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

if [[ "${list_maps}" == true ]]; then
  echo "地图编号（目录名；括号内依次为三位编号写法和兼容旧名称）："
  while IFS=$'\t' read -r map_folder legacy_name category; do
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
    while IFS=$'\t' read -r map_folder legacy_name category; do
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

runtime_map_parent=""
cleanup_runtime_map() {
  if [[ -n "${runtime_map_parent}" ]]; then
    rm -rf -- "${runtime_map_parent}"
  fi
}
trap cleanup_runtime_map EXIT

effective_map_scale="${requested_map_scale:-1.0}"
if [[ -z "${requested_map_scale}" && -n "${selected_map_dir}" ]]; then
  case "$(basename -- "${selected_map_dir%/}")" in
    map01-p11-narrow-case-a) effective_map_scale="0.75" ;;
    map03-p11-narrow-case-c) effective_map_scale="0.7" ;;
    map04-p12-maze-env1) effective_map_scale="0.5" ;;
    map05-p12-parking-row-env2) effective_map_scale="0.7" ;;
    map07-p12-obstacle-field-env4) effective_map_scale="0.8" ;;
    map08-p16-complex-parking) effective_map_scale="1.2" ;;
    map15-n06-two-stage-offset-passage) effective_map_scale="0.8" ;;
  esac
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

# The demo entry point follows the current planner unless the caller explicitly
# selects a backend. Keep this explicit so stale params.yaml values cannot make
# ordinary manual tests silently run the legacy planner.
has_planning_backend=false
for arg in "${launch_args[@]}"; do
  if [[ "${arg}" == planning_backend:=* ]]; then
    has_planning_backend=true
    break
  fi
done
if [[ "${has_planning_backend}" == false ]]; then
  launch_args+=("planning_backend:=mode_path_v2")
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
catkin_make -C "${overlay_dir}" -j4 >/dev/null
set +u
source "${overlay_dir}/devel/setup.bash"
set -u
bonminseed_plugin_dir="${overlay_dir}/devel/lib/fwis_v3_optimizer/casadi_plugins"
if [[ -f "${bonminseed_plugin_dir}/libcasadi_nlpsol_bonminseed.so" ]]; then
  export CASADIPATH="${bonminseed_plugin_dir}${CASADIPATH:+:${CASADIPATH}}"
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

python3 "${demo_dir}/tools/describe_run_context.py" \
  "${demo_dir}" "${selected_map_dir:-}" "${effective_map_scale}" "${launch_args[@]}"

echo "[expv3] 启动 Demo：${launch_args[*]}"
roslaunch fwis_v3_demo demo.launch "${launch_args[@]}"
