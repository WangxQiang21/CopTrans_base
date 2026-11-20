#!/usr/bin/env bash
set -u
D=/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK03_REWORK_02_evidence/20261120T025148Z/scan
run() {
  label=$1
  shift
  /usr/bin/time -f 'elapsed=%e exit=%x' -o "$D/$label.meta" \
    python3 "$D/scan_e09.py" --a 9 --b 11 --deg 20 --pad 1.5 \
    --label "$label" --seconds 60 "$@" > "$D/$label.log" 2>&1
}
run s07 --wall-x 9 --wall-y0 1.0 --wall-y1 2.5
run s08 --wall-x 9 --wall-y0 0.5 --wall-y1 2.5
run s09 --wall-x 9 --wall-y0 1.0 --wall-y1 3.0
run s10 --wall-x 12 --wall-y0 1.5 --wall-y1 3.5
run s11 --wall-x 12 --wall-y0 1.0 --wall-y1 3.0
run s12 --wall-x 6 --wall-y0 0.5 --wall-y1 2.0
