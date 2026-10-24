"""Durable geometry evidence when no execution certificate is available."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math

import numpy as np

from .continuous_geometry import replay_continuous
from .core import replay


class GeometryIncumbentStore:
    """Keep independent geometry snapshots; never label them executable."""

    def __init__(self, evaluator, max_records=4):
        self.evaluator = evaluator
        self.max_records = max_records
        self._records = []
        self._seen = set()
        self.attempts = []
        grid = evaluator.grid
        self.contract = {
            "map": {"occupancy_sha256": hashlib.sha256(
                np.asarray(grid.occ, dtype=np.uint8).tobytes()).hexdigest(),
                "width": grid.width, "height": grid.height,
                "resolution_m": grid.res, "origin": [grid.ox, grid.oy]},
            "physical_vehicle_m": [evaluator.length, evaluator.width],
            "margin_half_extents_m": [grid.half_l, grid.half_w],
            "sweep_spacing_m": evaluator.profile.sweep_spacing,
            "terminal_tolerance": {"xy_m": .02, "yaw_rad": float(np.deg2rad(1.))},
            "allow_shunting": evaluator.allow_shunting,
            "execution_profile_target": asdict(evaluator.limits),
        }

    @property
    def records(self):
        return deepcopy(self._records)

    def preserve(self, actions, source, parent=None, plan=None, audit=None,
                 reverse_allowed=None, found_at_s=None, execution_error=None):
        evaluator = self.evaluator
        if audit is None:
            audit = (evaluator.evaluate_continuous_geometry(
                plan, allow_reverse=reverse_allowed, include_metrics=False)
                if plan is not None else evaluator.evaluate(
                    actions, timed=None, allow_reverse=reverse_allowed))
        self.attempts.append({"source": source, "geometry_status": audit["geometry_status"],
                              "reason": audit.get("reason"),
                              "execution_error": execution_error})
        terminal = audit.get("terminal", {})
        xy = terminal.get("xy_error_m", terminal.get("xy_error", float("inf")))
        yaw = terminal.get("yaw_error_rad", terminal.get("yaw_error", float("inf")))
        if (audit["geometry_status"] != "PASS" or not math.isfinite(xy) or
                not math.isfinite(yaw) or xy > .02+1e-9 or yaw > math.radians(1)+1e-9):
            return False
        geometry = ({"representation": "continuous_motion_v2", "plan": asdict(plan)}
                    if plan is not None else
                    {"representation": "exact_actions_v1",
                     "start": asdict(evaluator.start), "goal": asdict(evaluator.goal),
                     "actions": [[a.mode, a.amount, a.control] for a in actions]})
        contract = dict(self.contract, allow_reverse=(evaluator.allow_reverse if
            reverse_allowed is None else bool(reverse_allowed)))
        encoded = json.dumps({"geometry": geometry, "contract": contract},
                             sort_keys=True, separators=(",", ":"), allow_nan=False)
        signature = hashlib.sha256(encoded.encode()).hexdigest()
        if signature in self._seen:
            return True
        if len(self._records) >= self.max_records:
            return True
        spatial = ([row[1] for row in replay_continuous(plan, evaluator.profile.sweep_spacing)] if plan is not None
                   else replay(evaluator.start, actions))
        geometry_audit = deepcopy(audit)
        geometry_audit.update(status="GEOMETRY_VALID", score=None, reason=None,
                              dynamic_status="NOT_CERTIFIED", timing_status="NOT_CERTIFIED")
        geometry_audit.pop("timing", None)
        self._records.append({
            "id": len(self._records), "geometry_sha256": signature,
            "source": source, "parent": parent, "found_at_s": found_at_s,
            "geometry": geometry, "contract": contract,
            "geometry_audit": geometry_audit,
            "spatial_path": [[p.x, p.y, p.th] for p in spatial],
            "execution_status": "NOT_CERTIFIED", "execution_error": execution_error,
            "publishable_as_timed_trajectory": False,
        })
        self._seen.add(signature)
        return True

    def report(self):
        records = self.records
        return {"geometry_candidates": records,
                "geometry_incumbent": records[0] if records else None,
                "geometry_retention_attempts": deepcopy(self.attempts),
                "first_geometry_certified_s": records[0]["found_at_s"] if records else None}
