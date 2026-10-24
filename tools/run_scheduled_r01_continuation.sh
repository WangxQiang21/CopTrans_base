#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/wq/whut_thesis/expv3
PROMPT_FILE="$PROJECT/qna/定时续跑任务_R01_20261009.md"
LOG_FILE="$PROJECT/relust/scheduled_r01_20261009.log"
RESULT_FILE="$PROJECT/relust/scheduled_r01_20261009_result.txt"
CRON_MARKER='# expv3-r01-20261009-once'
TARGET_LOCAL='2026-10-09 04:58:24'

remove_own_cron_entry() {
  local current filtered
  current="$(crontab -l 2>/dev/null || true)"
  filtered="$(printf '%s\n' "$current" | sed "\\|$CRON_MARKER|d")"
  if [[ -n "${filtered//[[:space:]]/}" ]]; then
    printf '%s\n' "$filtered" | crontab -
  else
    crontab -r 2>/dev/null || true
  fi
}

if [[ "${1:-}" == '--dry-run' ]]; then
  printf 'Target: %s Asia/Shanghai\n' "$TARGET_LOCAL"
  printf 'Prompt: %s\n' "$PROMPT_FILE"
  printf 'Project: %s\n' "$PROJECT"
  exit 0
fi

target_epoch="$(TZ=Asia/Shanghai date -d "$TARGET_LOCAL" +%s)"
target_day="$(TZ=Asia/Shanghai date -d "$TARGET_LOCAL" +%F)"
now_epoch="$(date +%s)"
today="$(TZ=Asia/Shanghai date +%F)"

if [[ "$today" != "$target_day" ]]; then
  remove_own_cron_entry
  exit 0
fi

if (( now_epoch < target_epoch )); then
  sleep "$((target_epoch - now_epoch))"
fi

remove_own_cron_entry
mkdir -p "$PROJECT/relust"
cd "$PROJECT"
exec /home/wq/.local/bin/codex --ask-for-approval never exec \
  --model gpt-6.1-sol \
  --config model_reasoning_effort=medium \
  --skip-git-repo-check \
  --sandbox workspace-write \
  --cd "$PROJECT" \
  --output-last-message "$RESULT_FILE" \
  - < "$PROMPT_FILE" >> "$LOG_FILE" 2>&1
