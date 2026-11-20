#!/usr/bin/env bash
set -u
D=/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK03_REWORK_02_evidence/20261120T025148Z/scan
run() {
  label=$1
  half=$2
  y0=$3
  y1=$4
  /usr/bin/time -f 'elapsed=%e exit=%x' -o "$D/$label.meta" \
    python3 "$D/scan_e09.py" --a 9 --b 11 --deg 20 --pad 1.5 \
    --wall-x 9 --wall-y0 "$y0" --wall-y1 "$y1" --wall-half "$half" \
    --label "$label" --seconds 60 > "$D/$label.log" 2>&1
}
run s13 1.0 0.5 2.5
run s14 2.0 0.5 2.5
run s15 3.0 0.5 2.5
run s16 1.0 1.5 4.0
run s17 2.0 1.5 4.0
run s18 3.0 1.5 4.0
