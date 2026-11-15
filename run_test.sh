#!/usr/bin/env bash
# 编队 V0.2（uniform_v02）地图测试启动脚本。
#
# 形式沿用 expv3/run_demo.sh：选地图编号、可选缩放比例、其余参数原样传给
# roslaunch。区别是本脚本针对当前 worktree 的编队 demo：
#   * 地图用编号指定，如 4 / 04 / map04 / map004；
#   * 缩放比例是绝对比例，省略时沿用 run_demo.sh 的编队默认（单车默认 x 3.75）；
#   * 若环境里没有 ROS_MASTER_URI，自动在私有空闲端口启动 roscore，
#     退出时只停止本次脚本启动的 master；
#   * 实际启动仍交给仓库根目录的 run_demo.sh，不复制其逻辑。
#
# 示例：
#   ./run_test.sh --list-maps
#   ./run_test.sh --map 4 --scale 0.5
#   ./run_test.sh 16 1.0
#   ./run_test.sh 1 rviz:=false
#   ./run_test.sh --map map01 --scale 0.75 target_v:=0.8
#   ./run_test.sh --dry-run --map 6 --scale 1.0
#
# Ctrl-C 结束 roslaunch；脚本随后停止自己启动的 roscore。
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
map_root="${repo_dir}/maps"
map_index="${map_root}/map_index.tsv"
demo_script="${repo_dir}/run_demo.sh"

usage() {
  cat <<'EOF'
用法：
  ./run_test.sh --list-maps
  ./run_test.sh <地图编号> [缩放比例] [ROS参数...]
  ./run_test.sh --map <编号|目录|旧名称> [--scale 比例] [ROS参数...]

地图编号：
  1-9、15、16 对应 maps/ 下当前可运行地图的编号；也接受 01、map04、
  map004、完整目录名（如 map04-p12-maze-env1）或旧名称（如 P12_env1）。

缩放比例：
  --scale、--map-scale、-scale 或第二个位置参数，均为绝对比例，
  以地图中心为基准且必须大于 0；省略时使用 run_demo.sh 的编队默认
  （地图表中单车默认比例 x 3.75）。

其余参数：
  含 := 的参数（如 rviz:=false、target_v:=0.8、auto_goal:=false）原样
  传给 roslaunch；--dry-run 只打印实际 roslaunch 命令。
  --no-master 不自动启动私有 roscore（要求 ROS_MASTER_URI 已可达）。
  --port 端口 指定自动启动 roscore 的端口（默认取空闲端口）。

示例：
  ./run_test.sh --list-maps
  ./run_test.sh --map 4 --scale 0.5
  ./run_test.sh 1 0.75
  ./run_test.sh 16 1.0 rviz:=true
  ./run_test.sh --map P12_env1 --scale 0.5 --port 11485
EOF
}

default_scale_for() {
  case "$1" in
    map01-p11-narrow-case-a) echo 0.75 ;;
    map03-p11-narrow-case-c) echo 0.7 ;;
    map04-p12-maze-env1) echo 0.5 ;;
    map05-p12-parking-row-env2) echo 0.7 ;;
    map07-p12-obstacle-field-env4) echo 0.8 ;;
    map08-p16-complex-parking) echo 1.2 ;;
    map15-n06-two-stage-offset-passage) echo 0.8 ;;
    *) echo 1.0 ;;
  esac
}

map_number_of() {
  local folder="$1"
  if [[ "${folder}" =~ ^map([0-9]{2})- ]]; then
    printf '%d' "$((10#${BASH_REMATCH[1]}))"
  else
    printf ''
  fi
}

list_maps() {
  printf '%-4s %-40s %-32s %-10s %-12s\n' \
    '编号' '目录' '旧名称' '单车默认' '编队默认'
  while IFS=$'\t' read -r folder legacy_name category; do
    [[ -z "${folder}" || "${folder}" == \#* ]] && continue
    [[ -f "${map_root}/${folder}/map.yaml" ]] || continue
    local number single formation
    number="$(map_number_of "${folder}")"
    single="$(default_scale_for "${folder}")"
    formation="$(python3 -c 'print(float(__import__("sys").argv[1]) * 3.75)' "${single}")"
    printf '%-4s %-40s %-32s %-10s %-12s\n' \
      "${number}" "${folder}" "${legacy_name}" "${single}" "${formation}"
  done < "${map_index}"
  echo
  echo "编队默认 = 单车默认 x 3.75（run_demo.sh 的旧策略，尚待按新构型重新验收）。"
  echo "显式 --scale 为绝对比例，不会再乘 3.75。"
}

resolve_map_dir() {
  local query="$1"
  local key="${query}"
  if [[ -d "${query}" ]]; then
    (cd -- "${query}" && pwd)
    return 0
  fi
  if [[ "${key}" =~ ^[0-9]{1,3}$ ]]; then
    key="$(printf 'map%02d' "$((10#${key}))")"
  elif [[ "${key}" =~ ^map0*([0-9]+)$ ]]; then
    key="$(printf 'map%02d' "$((10#${BASH_REMATCH[1]}))")"
  fi
  if [[ -d "${map_root}/${query}" ]]; then
    printf '%s\n' "${map_root}/${query}"
    return 0
  fi
  local folder legacy_name category
  while IFS=$'\t' read -r folder legacy_name category; do
    [[ -z "${folder}" || "${folder}" == \#* ]] && continue
    if [[ "${folder}" == "${key}" || "${folder}" == "${key}-"* || \
          "${legacy_name}" == "${query}" ]]; then
      [[ -f "${map_root}/${folder}/map.yaml" ]] || continue
      printf '%s\n' "${map_root}/${folder}"
      return 0
    fi
  done < "${map_index}"
  return 1
}

validate_scale() {
  python3 -c 'import math,sys; x=float(sys.argv[1]); sys.exit(0 if math.isfinite(x) and x>0 else 1)' "$1"
}

pick_free_port() {
  python3 - <<'PY'
import socket
s = socket.socket()
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])
s.close()
PY
}

port_reachable() {
  python3 - "$1" "$2" <<'PY'
import socket, sys
host, port = sys.argv[1], int(sys.argv[2])
try:
    with socket.create_connection((host, port), 0.5):
        pass
except OSError:
    sys.exit(1)
PY
}

uri_host_port() {
  python3 - "$1" <<'PY'
import sys, urllib.parse
u = urllib.parse.urlparse(sys.argv[1])
print(u.hostname or "127.0.0.1", u.port or 11311)
PY
}

requested_map=""
requested_scale=""
use_master=true
master_port="${TEST_MASTER_PORT:-}"
positional=()
passthrough=()
while (($#)); do
  case "$1" in
    --list-maps)
      list_maps
      exit 0
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --map|-m)
      (($# < 2)) && { echo "错误：--map 需要地图编号或名称。" >&2; exit 2; }
      [[ -n "${requested_map}" ]] && { echo "错误：--map 只能指定一次。" >&2; exit 2; }
      requested_map="$2"
      shift 2
      ;;
    --map=*)
      [[ -n "${requested_map}" ]] && { echo "错误：--map 只能指定一次。" >&2; exit 2; }
      requested_map="${1#--map=}"
      shift
      ;;
    --map-scale|--scale|-scale)
      (($# < 2)) && { echo "错误：缩放参数需要一个比例。" >&2; exit 2; }
      [[ -n "${requested_scale}" ]] && { echo "错误：缩放参数只能指定一次。" >&2; exit 2; }
      requested_scale="$2"
      shift 2
      ;;
    --map-scale=*|--scale=*|-scale=*)
      [[ -n "${requested_scale}" ]] && { echo "错误：缩放参数只能指定一次。" >&2; exit 2; }
      requested_scale="${1#*=}"
      shift
      ;;
    --no-master)
      use_master=false
      shift
      ;;
    --port)
      (($# < 2)) && { echo "错误：--port 需要一个端口号。" >&2; exit 2; }
      master_port="$2"
      shift 2
      ;;
    --dry-run)
      passthrough+=("$1")
      shift
      ;;
    --)
      shift
      while (($#)); do passthrough+=("$1"); shift; done
      ;;
    -*)
      echo "错误：未知选项 $1（ROS 参数请写成 name:=value）。" >&2
      exit 2
      ;;
    *:=*)
      passthrough+=("$1")
      shift
      ;;
    *)
      positional+=("$1")
      shift
      ;;
  esac
done
if ((${#positional[@]} > 0)); then
  [[ -n "${requested_map}" ]] && { echo "错误：地图编号既给了位置参数又给了 --map。" >&2; exit 2; }
  requested_map="${positional[0]}"
fi
if ((${#positional[@]} > 1)); then
  [[ -n "${requested_scale}" ]] && { echo "错误：缩放比例给了两次。" >&2; exit 2; }
  requested_scale="${positional[1]}"
fi
if ((${#positional[@]} > 2)); then
  echo "错误：无法识别的多余参数：${positional[*]:2}（ROS 参数请写成 name:=value）。" >&2
  exit 2
fi

[[ -f "${demo_script}" ]] || { echo "错误：找不到 ${demo_script}。" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "错误：需要 python3。" >&2; exit 1; }

if [[ -z "${requested_map}" ]]; then
  usage >&2
  echo "错误：必须指定地图编号（或用 --list-maps 查看）。" >&2
  exit 2
fi
if ! selected_map_dir="$(resolve_map_dir "${requested_map}")"; then
  echo "错误：找不到地图 ${requested_map}；用 ./run_test.sh --list-maps 查看。" >&2
  exit 2
fi
if [[ ! -f "${selected_map_dir}/map.yaml" ]]; then
  echo "错误：地图目录缺少 map.yaml：${selected_map_dir}" >&2
  exit 2
fi
if [[ -n "${requested_scale}" ]] && ! validate_scale "${requested_scale}"; then
  echo "错误：缩放比例必须是大于 0 的有限数值：${requested_scale}" >&2
  exit 2
fi

demo_args=("--map" "${selected_map_dir}")
[[ -n "${requested_scale}" ]] && demo_args+=("--scale" "${requested_scale}")
demo_args+=("${passthrough[@]}")

master_pid=""
cleanup() {
  if [[ -n "${master_pid}" ]]; then
    kill -INT "${master_pid}" 2>/dev/null || true
    wait "${master_pid}" 2>/dev/null || true
    master_pid=""
    echo "[run_test] 已停止本次启动的私有 roscore。"
  fi
}
trap cleanup EXIT

# --dry-run 交给 run_demo.sh 完成（它只打印命令，不 source/构建/启动）。
for arg in "${passthrough[@]}"; do
  if [[ "${arg}" == "--dry-run" ]]; then
    exec "${demo_script}" "${demo_args[@]}"
  fi
done

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
else
  echo "错误：未找到 ROS Noetic (/opt/ros/noetic/setup.bash)。" >&2
  exit 1
fi

using_existing_master=false
if [[ -n "${ROS_MASTER_URI:-}" ]]; then
  read -r master_host master_port_used < <(uri_host_port "${ROS_MASTER_URI}")
  if port_reachable "${master_host}" "${master_port_used}"; then
    using_existing_master=true
    echo "[run_test] 使用已有 ROS master：${ROS_MASTER_URI}"
  elif [[ "${use_master}" == true ]]; then
    echo "[run_test] 警告：ROS_MASTER_URI=${ROS_MASTER_URI} 不可达，改用自动启动的私有 master。" >&2
    unset ROS_MASTER_URI
  else
    echo "错误：ROS_MASTER_URI=${ROS_MASTER_URI} 不可达，且指定了 --no-master。" >&2
    exit 1
  fi
fi
if [[ "${using_existing_master}" == false ]]; then
  if [[ "${use_master}" == true ]]; then
  [[ -n "${master_port}" ]] || master_port="$(pick_free_port)"
  if port_reachable 127.0.0.1 "${master_port}"; then
    echo "错误：端口 ${master_port} 已被占用，请换 --port。" >&2
    exit 1
  fi
  export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
  export ROS_HOSTNAME=127.0.0.1
  export ROS_IP=127.0.0.1
  export ROS_HOME="${TEST_ROS_HOME:-$(mktemp -d "${TMPDIR:-/tmp}/expv3-test-ros.XXXXXX")}"
  mkdir -p "${ROS_HOME}"
  echo "[run_test] 启动私有 roscore：${ROS_MASTER_URI}（ROS_HOME=${ROS_HOME}）"
  roscore -p "${master_port}" > "${ROS_HOME}/roscore_outer.log" 2>&1 &
  master_pid=$!
  ready=false
  for _ in $(seq 1 100); do
    if port_reachable 127.0.0.1 "${master_port}"; then
      ready=true
      break
    fi
    if ! kill -0 "${master_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if [[ "${ready}" != true ]]; then
    echo "错误：私有 roscore 未能启动，日志：${ROS_HOME}/roscore_outer.log" >&2
    exit 1
  fi
  else
    echo "错误：--no-master 需要已有 ROS_MASTER_URI。" >&2
    exit 1
  fi
fi

echo "[run_test] 地图：${selected_map_dir}"
if [[ -n "${requested_scale}" ]]; then
  echo "[run_test] 缩放：${requested_scale}（绝对比例）"
else
  echo "[run_test] 缩放：未指定，沿用 run_demo.sh 的编队默认（见 --list-maps）。"
fi
echo "[run_test] 启动编队 V0.2 demo；RViz 中用 2D Pose Estimate / 2D Nav Goal 设置起终点。"
"${demo_script}" "${demo_args[@]}"