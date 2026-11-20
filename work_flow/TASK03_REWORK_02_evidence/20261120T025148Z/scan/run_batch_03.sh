#!/usr/bin/env bash
set -u
D=/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK03_REWORK_02_evidence/20261120T025148Z/scan
run() {
  label=$1
  deg=$2
  top=$3
  half=$4
  /usr/bin/time -f 'elapsed=%e exit=%x' -o "$D/$label.meta" \
    python3 "$D/scan_e09.py" --a 9 --b 11 --deg "$deg" --pad 1.5 \
    --wall-x 9 --wall-y0 -10 --wall-y1 "$top" --wall-half "$half" \
    --label "$label" --seconds 60 > "$D/$label.log" 2>&1
}
run s19 0 2.5 1.0
run s20 0 2.5 2.0
run s21 0 4.0 1.0
run s22 0 4.0 2.0
run s23 10 3.5 1.0
run s24 10 3.5 2.0
