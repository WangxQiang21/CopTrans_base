"""T5 front-end termination, exact-connection orders and consecutive goals."""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"))

from fwis_v3_optimizer.formation_geometry import (
    BODY_SPIN,
    build_continuous_plans,
)
from fwis_v3_optimizer.formation_kinematics import (
    ROTATION,
    TRANSLATION,
)
from fwis_v3_optimizer.formation_quality import FEASIBLE, REJECTED
from fwis_v3_optimizer.formation_request import validate_formation_request

from formation_optimizer_node import FormationPlanningPipeline
from tests.test_formation_optimizer_node import (
    pipeline_config,
    primitive,
    publishable_core,
    seed_message,
    straight_request,
)
from tests.test_formation_t3_missions import (
    empty_grid as wedge_empty_grid,
    make_request as wedge_request,
)


def wedge_seed(request, primitives):
    return SimpleNamespace(
        header=request.header, plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=tuple(primitives))


def frontend_report(request, node_state, reason, detail="", valid=True,
                    field=""):
    return {
        "schema": "formation_frontend_status/v1",
        "valid": valid,
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "node_state": node_state,
        "reason": reason,
        "field": field,
        "detail": detail,
        "trajectory_published": False,
        "executable": False,
    }


class T5FrontendTransactionTest(unittest.TestCase):

    def test_frontend_failure_ends_request_and_rejects_late_seed(self):
        core = publishable_core()
        request = straight_request(plan_id=51)
        event = core.on_request(request, publication_stamp=(2000, 0))
        self.assertEqual(event.status["node_state"], "REQUEST_VALIDATED")

        failed = core.on_frontend_status(
            frontend_report(request, "GEOMETRY_SEARCH_FAILED",
                            "NO_GEOMETRY_PATH",
                            "search budget exhausted without a seed"),
            publication_stamp=(2001, 0))
        self.assertIsNotNone(failed)
        self.assertEqual(failed.status["node_state"], "FRONTEND_FAILED")
        self.assertEqual(failed.status["status"], REJECTED)
        self.assertEqual(failed.status["reason"], "NO_GEOMETRY_PATH")
        self.assertTrue(failed.clear)
        self.assertFalse(failed.status["trajectory_published"])
        self.assertFalse(failed.status["executable"])
        self.assertIsNone(core.gate.published_plan_id)

        # A seed that was already on the wire must not revive a failed plan.
        seed = seed_message(request, (primitive(0, dx=2.0),))
        late = core.on_seed(seed, publication_stamp=(2002, 0))
        self.assertEqual(late.status["node_state"], "SEED_REJECTED")
        self.assertEqual(late.status["reason"], "FRONTEND_FAILED")
        self.assertFalse(late.status["trajectory_published"])
        self.assertIsNone(core.gate.published_plan_id)

        # The next request is independent and may publish normally.
        newer = straight_request(plan_id=52)
        core.on_request(newer, publication_stamp=(2003, 0))
        published = core.on_seed(
            seed_message(newer, (primitive(0, dx=2.0),)),
            publication_stamp=(2004, 0))
        self.assertEqual(published.status["node_state"],
                         "FEASIBLE_PUBLISHED")
        self.assertEqual(core.gate.published_plan_id, 52)

    def test_frontend_failure_arriving_first_still_ends_the_request(self):
        core = publishable_core()
        request = straight_request(plan_id=55)
        # The status and request topics are independent; the status callback
        # can be delivered before the request callback.
        self.assertIsNone(core.on_frontend_status(
            frontend_report(request, "GEOMETRY_SEARCH_FAILED",
                            "NO_GEOMETRY_PATH"),
            publication_stamp=(2005, 0)))
        event = core.on_request(request, publication_stamp=(2006, 0))
        self.assertEqual(event.status["node_state"], "FRONTEND_FAILED")
        self.assertEqual(event.status["status"], REJECTED)
        self.assertEqual(event.status["reason"], "NO_GEOMETRY_PATH")
        self.assertFalse(event.status["trajectory_published"])
        self.assertFalse(event.status["executable"])
        self.assertIsNone(core.gate.published_plan_id)

        late = core.on_seed(
            seed_message(request, (primitive(0, dx=2.0),)),
            publication_stamp=(2007, 0))
        self.assertEqual(late.status["node_state"], "SEED_REJECTED")
        self.assertEqual(late.status["reason"], "FRONTEND_FAILED")

    def test_transient_frontend_map_unavailable_does_not_kill_the_seed(self):
        core = publishable_core()
        request = straight_request(plan_id=56)
        core.on_request(request, publication_stamp=(2008, 0))
        # The C++ front end publishes MAP_UNAVAILABLE when the request wins
        # the race against its latched map, then keeps the request pending and
        # publishes the seed when the map arrives.
        self.assertIsNone(core.on_frontend_status(
            frontend_report(request, "MAP_UNAVAILABLE", "MAP_UNAVAILABLE"),
            publication_stamp=(2009, 0)))
        event = core.on_seed(
            seed_message(request, (primitive(0, dx=2.0),)),
            publication_stamp=(2010, 0))
        self.assertEqual(event.status["node_state"], "FEASIBLE_PUBLISHED")
        self.assertEqual(core.gate.published_plan_id, 56)

    def test_frontend_success_and_stale_failure_are_ignored(self):
        core = publishable_core()
        request = straight_request(plan_id=53)
        core.on_request(request, publication_stamp=(2010, 0))
        success = frontend_report(
            request, "GEOMETRY_ONLY_UNCERTIFIED", "", valid=True)
        self.assertIsNone(core.on_frontend_status(
            success, publication_stamp=(2011, 0)))
        self.assertIsNone(core.gate.published_plan_id)

        core.on_request(straight_request(plan_id=54),
                        publication_stamp=(2012, 0))
        stale = frontend_report(
            request, "GEOMETRY_SEARCH_FAILED", "NO_GEOMETRY_PATH")
        self.assertIsNone(core.on_frontend_status(
            stale, publication_stamp=(2013, 0)))
        self.assertEqual(core.snapshot.plan_id, 54)
        self.assertIsNone(core._frontend_failed_plan_id)

    def test_consecutive_goals_old_result_cannot_override_new(self):
        core = publishable_core()
        first = straight_request(plan_id=61)
        core.on_request(first, publication_stamp=(2020, 0))
        first_seed = seed_message(first, (primitive(0, dx=2.0),))
        first_event = core.on_seed(first_seed, publication_stamp=(2021, 0))
        self.assertEqual(first_event.status["node_state"],
                         "FEASIBLE_PUBLISHED")
        self.assertEqual(core.gate.published_plan_id, 61)

        second = straight_request(plan_id=62)
        second_request = core.on_request(
            second, publication_stamp=(2022, 0))
        self.assertEqual(second_request.status["node_state"],
                         "REQUEST_VALIDATED")
        self.assertTrue(second_request.clear)
        self.assertIsNone(core.gate.published_plan_id)

        stale = core.on_seed(first_seed, publication_stamp=(2023, 0))
        self.assertEqual(stale.status["node_state"], "SEED_STALE")
        self.assertFalse(stale.status["trajectory_published"])
        self.assertIsNone(core.gate.published_plan_id)

        second_seed = seed_message(second, (primitive(0, dx=1.0),))
        second_event = core.on_seed(second_seed, publication_stamp=(2024, 0))
        self.assertEqual(second_event.status["node_state"],
                         "FEASIBLE_PUBLISHED")
        self.assertEqual(core.gate.published_plan_id, 62)

        # A late duplicate of the old plan cannot replace plan 62.
        late = core.on_seed(first_seed, publication_stamp=(2025, 0))
        self.assertEqual(late.status["node_state"], "SEED_STALE")
        self.assertFalse(late.status["trajectory_published"])
        self.assertEqual(core.gate.published_plan_id, 62)


class T5ExactConnectionTest(unittest.TestCase):

    def _mixed_request(self):
        return wedge_request((13.0, 10.0, math.pi / 2.0), plan_id=8100)

    def test_mixed_translation_and_cargo_rotation_audits_both_orders(self):
        request = self._mixed_request()
        snapshot = validate_formation_request(request).snapshot
        # The S4 front end's translate-first connection: 3 m along x, then a
        # +90 deg cargo rotation about the cargo point.
        seed = wedge_seed(request, (
            SimpleNamespace(
                geometry_type=TRANSLATION, dx=3.0, dy=0.0, dpsi=0.0,
                pivot_x=0.0, pivot_y=0.0),
            SimpleNamespace(
                geometry_type=ROTATION, dx=0.0, dy=0.0,
                dpsi=math.pi / 2.0, pivot_x=0.0, pivot_y=0.0),
        ))
        build = build_continuous_plans(
            snapshot, seed, max_candidates=4,
            max_schedule_combinations=64, max_schedules_per_block=4)
        self.assertTrue(build.success, build.detail)
        self.assertIn("EXACT_CONNECTION_ORDERS_AUDITED=2", build.detail)
        signatures = set()
        for plan in build.plans:
            signatures.add(tuple(
                block.geometry_type for block in plan.blocks))
        self.assertTrue(any(
            signature and signature[-1] == TRANSLATION
            for signature in signatures), signatures)
        self.assertTrue(any(
            BODY_SPIN in signature or ROTATION in signature
            for signature in signatures), signatures)

        result = FormationPlanningPipeline(replace(
            pipeline_config(), max_candidates=4)).run(
                snapshot, seed, wedge_empty_grid())
        self.assertEqual(result.status, FEASIBLE, result.detail)
        self.assertIsNotNone(result.selection)
        selected = result.selection.selected_plan
        modes = tuple(block.geometry_type for block in selected.blocks)
        self.assertIn(TRANSLATION, modes)
        self.assertIn(ROTATION, modes)
        self.assertIn(BODY_SPIN, modes)

    def test_goal_collision_is_an_explicit_rejection_not_a_timeout(self):
        request = wedge_request((15.0, 10.0, 0.0), plan_id=8101)
        snapshot = validate_formation_request(request).snapshot
        seed = wedge_seed(request, (SimpleNamespace(
            geometry_type=TRANSLATION, dx=5.0, dy=0.0, dpsi=0.0,
            pivot_x=0.0, pivot_y=0.0),))
        grid = wedge_empty_grid()
        # car 1 goal centre is (16, 10); occupy the goal cell without
        # blocking the cargo centre itself.
        grid.occ[200, 320] = True
        result = FormationPlanningPipeline(pipeline_config()).run(
            snapshot, seed, grid)
        self.assertNotEqual(result.status, FEASIBLE)
        self.assertEqual(result.status, REJECTED)
        self.assertIsNone(result.selection)
        self.assertTrue(result.rejections)
        status = result.as_status()
        self.assertFalse(status["trajectory_published"])
        self.assertFalse(status["executable"])


if __name__ == "__main__":
    unittest.main()