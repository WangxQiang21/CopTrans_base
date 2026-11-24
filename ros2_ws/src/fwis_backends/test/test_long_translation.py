"""Long direct A/O connections retain the physical per-block duration limit."""

import numpy as np
import pytest

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig


@pytest.mark.parametrize(("goal_y", "mode"), [(4.0, "A"), (7.0, "O")])
def test_low_speed_long_translation_stays_within_block_limit(goal_y, mode):
    grid = Grid(np.zeros((220, 510), dtype=bool), 0.05, 0.0, 0.0)
    model = FormationModel([VehicleOffset(1, 1.0, 0.0),
                            VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    goal = FormationPose(21.0, goal_y, 0.0)
    limits = FormationTimingLimits(v_max=0.16, a_max=0.22, r_min_ackermann=0.90)
    request = UniformPlanningRequest(
        grid, model, dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5)),
        FormationPose(3.0, 4.0, 0.0), dict.fromkeys(model.vehicle_ids, 0.0), goal,
        search_config=GraphSearchConfig(direct_goal_translation=True),
        successor_config=SuccessorConfig(timing_limits=limits),
        cargo_footprint=CargoFootprint(1.95, 2.70, 0.5, 0.0))

    result = plan_uniform(request)
    assert result.executable, (result.reason, result.detail)
    terminal = result.plan.goal_qf
    assert (terminal.x, terminal.y, terminal.psi) == pytest.approx((goal.x, goal.y, goal.psi))
    motion = [phase for phase in result.plan.phases if phase.kind == "motion"]
    assert len(motion) > 1
    assert all(phase.mode == mode and phase.duration_s <= limits.max_block_duration_s for phase in motion)
    assert result.plan.duration_s > limits.max_block_duration_s
