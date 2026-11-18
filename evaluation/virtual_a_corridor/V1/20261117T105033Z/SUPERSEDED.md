# SUPERSEDED

This run captured valid corrected-scale screenshots and a 7-message recorder
log, but its recorder saved only point counts, z-range and color.  Codex
acceptance requires comparing published vertices with the C1 JSON, so this run
was superseded by `../20261117T105607Z/`, whose recorder stores every
`point_xyz` and `Marker.scale` and whose `verify_corridor_markers.py` proves
exact vertex equality and the fill scale fix.

Differences worth knowing:

- This run's CLEARED fixture re-sent the matching `plan_request` first, so the
  recorder shows two DELETEALL messages in a row (request-admission clear at
  message 4, CLEARED at message 5).  The superseding run uses
  `--skip-request --skip-job` for the CLEARED case, giving exactly one
  DELETEALL message.
- The startup env error seen while creating the superseding run does not
  affect this run.

The screenshots and message composition here are not contradicted by the
superseding run; keep them as supporting evidence, not as the primary record.