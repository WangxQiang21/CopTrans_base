"""Independently certify a legacy front-end proposal on the lattice snapshot."""

from .mode_master import ModePathMaster
from .quality import TrajectoryEvaluator
from .state_lattice import Candidate, audited_continuous_candidate


def optimize_legacy_seed(seed, start, goal, grid, limits, footprint, config, profile, *,
                         analytic_ackermann):
    evaluator = TrajectoryEvaluator(grid, start, goal, limits, *footprint,
                                    xy_tol=.02, yaw_tol=.017453292519943295, profile=profile,
                                    allow_reverse=config.allow_reverse, allow_shunting=False)
    winner, _report = ModePathMaster(evaluator, config).solve(seed)
    geometry = winner.get("continuous_plan")
    actions = tuple(winner["actions"])
    if geometry is None or winner["audit"]["status"] != "FEASIBLE":
        return None
    candidate = Candidate(actions, winner["timed"], winner["audit"], geometry)
    return audited_continuous_candidate(geometry, actions, evaluator,
                                        analytic_ackermann=analytic_ackermann, existing=candidate)
