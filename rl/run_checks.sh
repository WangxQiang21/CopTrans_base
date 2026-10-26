#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir=""
with_ros=false
while (($#)); do
  case "$1" in
    --output)
      output_dir="${2:?--output requires a directory}"
      shift 2
      ;;
    --ros)
      with_ros=true
      shift
      ;;
    *)
      echo "Usage: bash rl/run_checks.sh --output NEW_DIRECTORY [--ros]" >&2
      exit 2
      ;;
  esac
done
if [[ -z "$output_dir" || -e "$output_dir" ]]; then
  echo "--output must name a new directory." >&2
  exit 2
fi
if [[ ! -x "$root_dir/.venv-rl/bin/python" ]]; then
  echo "Create .venv-rl first; see rl/README.md." >&2
  exit 2
fi
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"
cd "$root_dir"
export PYTHONPATH="$root_dir/rl/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s tests -p 'test_*.py' 2>&1 | tee "$output_dir/kernel-tests.txt"
.venv-rl/bin/python -m unittest discover -s rl/tests -p 'test_*.py' 2>&1 | tee "$output_dir/environment-tests.txt"
.venv-rl/bin/python -m coptrans_rl.foundation --output "$output_dir/foundation" | tee "$output_dir/foundation.txt"
python3 rl/tests/kernel_contract_snapshot.py --output "$output_dir/kernel-ros.json"
.venv-rl/bin/python rl/tests/kernel_contract_snapshot.py --output "$output_dir/kernel-learning.json"
.venv-rl/bin/python - "$output_dir" <<'PY'
import json, math, sys
from pathlib import Path
p=Path(sys.argv[1])
a=json.loads((p/'kernel-ros.json').read_text())
b=json.loads((p/'kernel-learning.json').read_text())
errors=[]
def compare(x,y):
    if isinstance(x,dict):
        assert x.keys()==y.keys()
        for k in x: compare(x[k],y[k])
    elif isinstance(x,(int,float)):
        errors.append(abs(x-y))
    else: assert x==y
compare(a['cases'],b['cases'])
assert a['contract_sha256']==b['contract_sha256']
maximum=max(errors)
assert math.isfinite(maximum) and maximum<1e-6, maximum
report={'passed':True,'maximum_absolute_numeric_error':maximum,
        'ros_runtime':a['runtime'],'learning_runtime':b['runtime']}
(p/'kernel-parity.json').write_text(json.dumps(report,indent=2)+'\n')
print('Cross-runtime common-kernel parity:',report)
PY
if $with_ros; then
  set +u
  source /opt/ros/noetic/setup.bash
  source "$root_dir/catkin_ws/devel/setup.bash"
  set -u
  python3 rl/tests/ros_playback_audit.py --input "$output_dir/foundation/native_timing.json" --output "$output_dir/ros-playback" | tee "$output_dir/ros-playback.txt"
fi
echo "Results: $output_dir"
