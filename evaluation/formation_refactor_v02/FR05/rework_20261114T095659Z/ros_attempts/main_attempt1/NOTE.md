# main attempt 1: failed on s_to_a interior ticks

This attempt ran while the FR04 S09 200-state unit measurement was still
running in parallel (CPU contention). The s_to_a plan was found and played
(T_end 21.443573 s, model_sha256 fdfd0e5c…), but the display loop only reached
about 53 Hz and the playback received the 1078-sample envelope ~0.15 s after
its publication stamp, so the 0.3 s initial_setup phase produced 9 interior
ticks instead of the required 11. All other main checks (including the strict
request-goal tolerance check) were green until that check.

The attempt is preserved unchanged; the next attempt reruns on an idle host.
