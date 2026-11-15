# Near-goal scenario, attempt 1 (preserved failure)

- Result: `ok = False`, `error = SmokeError('envelope_certification: ')`.
- The planner honestly failed plan 600004 after the frozen 30 s budget:
  `not executable: status=BUDGET_EXHAUSTED reason=TIME_BUDGET_EXHAUSTED
   layer=budget wall=30.112s`, then published the invalid placeholder
  envelope that clears playback.
- The scenario loop treated that placeholder (same plan_id, empty model,
  empty certification) as a published envelope and ran the envelope audit on
  it. That is a harness defect, not a planner defect: `wait_envelope`
  already filters on `model_json` and samples, but this loop did not. The
  loop now checks job FAILED first and only accepts a populated envelope.
- The plan itself keeps running in the log; the next attempt is a rerun on
  the same demo (plan ids advance).
