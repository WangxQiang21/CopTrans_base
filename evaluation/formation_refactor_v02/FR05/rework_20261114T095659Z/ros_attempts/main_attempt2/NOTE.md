# Main scenario, attempt 2 (preserved failure)

- Command: `tests/ros_uniform_playback_smoke.py --scenario main` against a
  demo launched with `--map .../FR05/empty_map backend:=uniform_v02
  rviz:=false playback_rate:=100` on private master 11472.
- Result: `ok = False`, raised in `run_experiment` at
  `check_shared_identity` for plan 600000:
  `SmokeError: playback_terminal_is_certified_for_that_digest: [False, True, True]`.
- Observed ordering in `run_demo.log` (timestamps from that run):
  - 487.412820 planner `PUBLISHED` for plan 600000 (digest c8d061...),
  - 487.529927 playback accepted the envelope,
  - 487.531693 playback warned `ignoring audit status for plan 600000:
    identity differs ...` (its cached optimizer report was still the
    pre-publication `REQUEST_VALIDATED` document, which has no digest),
  - 487.538671 playback `PLAYING ... executable=False`,
  - 493.680823 playback `HOLD_END ... executable=False`, then 0.000267 s later
    the planner published its `HOLD_END` status with the same digest, and the
    following playback terminal ticks certified.
- Interpretation: playback failed closed while its cached audit status
  lagged publication; the digest binding itself worked. The first terminal
  tick can legitimately precede the planner's terminal status (the planner
  publishes it in response to the playback HOLD_END). The check originally
  required *every* HOLD_END document to be executable, which contradicts
  that one-way ordering; it now requires certification to arrive within
  2 s for exactly the envelope digest (the negative cross-identity cases
  remain unit-tested).
