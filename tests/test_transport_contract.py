"""Authoritative cargo-transport contract and its plumbing into the launch."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ElementTree

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.transport_contract import (  # noqa: E402
    POLICY_CARGO_HEADING, POLICY_TANGENT, TransportContractError,
    contract_from_mapping, load_contract)

CONFIG = (ROOT / "catkin_ws/src/fwis_v3_demo/config/"
          "transport_formation.yaml")
LAUNCH = ROOT / "catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch"


def contract_values():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


class TransportContractTest(unittest.TestCase):

    def setUp(self):
        self.values = contract_values()
        self.contract = contract_from_mapping(self.values)

    def test_authoritative_ids_offsets_and_dimensions(self):
        self.assertEqual(self.contract.formation_id, "cargo_wedge")
        self.assertEqual(self.contract.vehicle_ids, (1, 2, 3))
        self.assertEqual(self.contract.offsets,
                         ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0)))
        self.assertAlmostEqual(self.contract.body_length_m, 0.738)
        self.assertAlmostEqual(self.contract.body_width_m, 0.500)
        self.assertTrue(self.contract.allow_reverse)
        self.assertTrue(self.contract.goal_heading_free)
        self.assertEqual(self.contract.initial_heading_policy,
                         POLICY_CARGO_HEADING)

    def test_fixed_wedge_distances_and_radii(self):
        offsets = self.contract.offset_map()
        self.assertAlmostEqual(math.dist(offsets[1], offsets[2]), math.sqrt(2))
        self.assertAlmostEqual(math.dist(offsets[1], offsets[3]), math.sqrt(2))
        self.assertAlmostEqual(math.dist(offsets[2], offsets[3]), 2.0)
        for vehicle_id in self.contract.vehicle_ids:
            self.assertAlmostEqual(math.hypot(*offsets[vehicle_id]), 1.0)

    def test_cargo_heading_policy_initialises_every_body_to_cargo(self):
        headings = self.contract.headings_for_policy(0.7)
        self.assertEqual(headings, {1: 0.7, 2: 0.7, 3: 0.7})

    def test_tangent_policy_is_the_analytic_rotation_baseline(self):
        values = dict(self.values)
        values["initial_heading_policy"] = POLICY_TANGENT
        tangent = contract_from_mapping(values)
        headings = tangent.headings_for_policy(0.0)
        expected = {1: math.pi / 2.0, 2: math.pi, 3: 0.0}
        for vehicle_id, angle in expected.items():
            self.assertAlmostEqual(
                math.atan2(math.sin(headings[vehicle_id] - angle),
                           math.cos(headings[vehicle_id] - angle)), 0.0)
        shifted = tangent.headings_for_policy(0.6)
        self.assertAlmostEqual(shifted[1], 0.6 + math.pi / 2.0)

    def test_contract_rejects_swapped_or_incomplete_values(self):
        for mutation in (
                {"offsets": [[1.0, 0.0], [0.0, 1.0]]},
                {"vehicle_ids": [1, 1, 3]},
                {"goal_heading_free": "true"},
                {"initial_heading_policy": "cargo"},
                {"body_length_m": 0.0}):
            values = dict(self.values)
            values.update(mutation)
            with self.assertRaises(TransportContractError):
                contract_from_mapping(values)

    def test_load_contract_requires_every_field(self):
        def params(name):
            return self.values[name]
        contract = load_contract(params)
        self.assertEqual(contract.offsets, self.contract.offsets)
        missing = dict(self.values)
        missing.pop("offsets")
        with self.assertRaises(KeyError):
            load_contract(lambda name: missing[name])


class TransportLaunchTest(unittest.TestCase):

    def setUp(self):
        self.root = ElementTree.parse(LAUNCH).getroot()

    def test_every_demo_node_loads_the_one_authoritative_config(self):
        args = {arg.get("name"): arg.get("default")
                for arg in self.root.findall("arg")}
        self.assertIn("transport_formation.yaml", args["transport_config"])
        self.assertTrue(CONFIG.is_file())
        nodes = {node.get("name"): node
                 for node in self.root.findall("node")}
        for name in ("formation_goal_bridge", "formation_playback"):
            loads = nodes[name].findall("rosparam")
            self.assertEqual(len(loads), 1, name)
            self.assertEqual(loads[0].get("command"), "load")
            self.assertEqual(loads[0].get("file"), "$(arg transport_config)")
            self.assertIsNone(loads[0].get("param"))

    def test_no_second_hard_coded_offset_list_survives(self):
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertNotIn("[0.0, 1.0], [-1.0, 0.0], [1.0, 0.0]", text)
        self.assertNotIn("- [-1.0, 0.0]", text)

    def test_cargo_heading_lock_is_explicitly_non_default(self):
        args = {arg.get("name"): arg.get("default")
                for arg in self.root.findall("arg")}
        self.assertEqual(args["require_cargo_heading"], "false")
        optimizer = next(node for node in self.root.findall("node")
                         if node.get("name") == "formation_optimizer")
        lock = optimizer.find(
            "param[@name='kinematic_limits/require_cargo_heading']")
        self.assertIsNotNone(lock)
        self.assertEqual(lock.get("value"),
                         "$(arg require_cargo_heading)")


if __name__ == "__main__":
    unittest.main()