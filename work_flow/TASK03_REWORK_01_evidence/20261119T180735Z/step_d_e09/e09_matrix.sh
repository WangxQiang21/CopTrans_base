#!/bin/bash
# E09 scene-feasibility matrix: does any certified union path leave the old
# allocation-fixed region while staying inside the adjacent audited Chain
# region?  Each log records CHAIN/GUIDE/ALLOC, captured template F segs and
# covers, per-sample r_alloc profile, and the old audit_path_in_corridors
# verdict on the captured path.
set -u
cd /home/wq/whut_thesis/expv3_multi_virtual_a || exit 1
OUT=/tmp/e09_sweep2
mkdir -p "$OUT"
run() {
  local name="$1"; shift
  echo "=== $name $*" > "$OUT/$name.log"
  timeout 240 python3 /tmp/e09_prof.py "$@" >> "$OUT/$name.log" 2>&1
  echo "EXIT $?" >> "$OUT/$name.log"
}
run m01_a18b30_d45_p3_fwd    --a 18 --b 30 --deg 45  --pad 3.0 --only forward
run m02_a18b30_d45_p3_bwd    --a 18 --b 30 --deg 45  --pad 3.0 --only backward
run m03_a18b18_d90_p3_fwd    --a 18 --b 18 --deg 90  --pad 3.0 --only forward
run m04_a18b18_d90_p3_bwd    --a 18 --b 18 --deg 90  --pad 3.0 --only backward
run m05_a12b24_d90_p3_fwd    --a 12 --b 24 --deg 90  --pad 3.0 --only forward
run m06_a24b12_d90_p3_fwd    --a 24 --b 12 --deg 90  --pad 3.0 --only forward
run m07_a18b18_d60_p3_fwd    --a 18 --b 18 --deg 60  --pad 3.0 --only forward
run m08_a18b18_d120_p3_fwd   --a 18 --b 18 --deg 120 --pad 3.0 --only forward
run m09_a8b20_d90_p35_fwd    --a 8  --b 20 --deg 90  --pad 3.5 --only forward
run m10_a20b8_d90_p35_fwd    --a 20 --b 8  --deg 90  --pad 3.5 --only forward
run m11_a18b18_d90_p25_fwd   --a 18 --b 18 --deg 90  --pad 2.5 --only forward
run m12_a18b18_d90_p3_gh0_fwd --a 18 --b 18 --deg 90 --pad 3.0 --goal-heading other --only forward
run m13_a18b18_d45_p4_fwd    --a 18 --b 18 --deg 45  --pad 4.0 --only forward
run m14_a18b18_d90_p3_psi45_fwd --a 18 --b 18 --deg 90 --pad 3.0 --psi0 0.785 --only forward
echo DONE