#!/usr/bin/env bash
# TASK03 rework end-state evidence.  Mirrors the recorded start snapshot
# methods: status porcelain, diff stat, protected hashes, evaluation tree,
# TASK01/TASK02 evidence tree hashes (find | sort | sha256sum | sha256sum).
set -u
REPO=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$REPO/work_flow/TASK03_REWORK_01_evidence/20261119T180735Z"
cd "$REPO" || exit 1

{
  echo "utc_end=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "head=$(git rev-parse HEAD)"
  echo "branch=$(git rev-parse --abbrev-ref HEAD)"
  echo "remotes:"
  git remote -v
  echo -n "left_right_vs_origin="
  git rev-list --left-right --count HEAD...@{upstream} 2>/dev/null || echo "no-upstream"
  echo "status_entries=$(git status --porcelain | wc -l)"
} > "$E/end/git_identity.txt"

git status --porcelain > "$E/end/git_status_porcelain.txt"
git diff --stat > "$E/end/git_diff_stat.txt"

sha256sum \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py \
  tests/test_virtual_a_geometry.py > "$E/end/protected_files.txt"

{
  echo "evaluation_status:"
  git status --porcelain -- evaluation
  echo "evaluation_head_tree:"
  git ls-tree -r HEAD -- evaluation | sha256sum
  echo "work_flow/TASK01_evidence tree:"
  find work_flow/TASK01_evidence -type f -print0 | sort -z | \
    xargs -0 sha256sum | sha256sum
  echo "work_flow/TASK01_evidence files: $(find work_flow/TASK01_evidence -type f | wc -l)"
  echo "work_flow/TASK02_evidence tree:"
  find work_flow/TASK02_evidence -type f -print0 | sort -z | \
    xargs -0 sha256sum | sha256sum
  echo "work_flow/TASK02_evidence files: $(find work_flow/TASK02_evidence -type f | wc -l)"
} > "$E/end/protected_dirs.txt"

# Rework-only diff against the extracted start snapshot, one recorded source
# path at a time (no unrelated tree is walked).
: > "$E/end/rework_only_diff.patch"
while IFS= read -r rel; do
  [[ -f "/tmp/rework01_start_snapshot/$rel" ]] || continue
  diff -u "/tmp/rework01_start_snapshot/$rel" "$REPO/$rel" \
    --label "a/$rel" --label "b/$rel" >> "$E/end/rework_only_diff.patch"
done < "$E/start/task03_sources.txt"
echo "rework_only_diff.patch lines: $(wc -l < "$E/end/rework_only_diff.patch")"
echo "changed files:"
grep '^+++ ' "$E/end/rework_only_diff.patch" | sed 's#^+++ b/##'
echo DONE