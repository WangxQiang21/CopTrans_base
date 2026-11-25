"""Endpoint and spawn regressions on the installed factory geometry, without starting ROS/Gazebo."""
import importlib.util
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from xml.etree import ElementTree

from launch import LaunchContext
import yaml

from cases import ROOT
from closed_loop import FAMILIES, fixture_config, fixture_preflight, fixture_scene, same_fixture


class FactoryFixtures(unittest.TestCase):
    def test_old_collision_and_new_relative_maneuvers(self):
        for family in ("bend", "lane_change", "diagonal"):
            with self.subTest(family=family):
                cfg = fixture_config(family, "formation", "standard")
                x, y, heading = FAMILIES[family]
                self.assertEqual(cfg["start"], [20.5, 3.5, 0.])
                self.assertEqual(cfg["goal"], [20.5+x, 3.5+y, heading])
                scene = fixture_scene(cfg, "formation")
                proof = fixture_preflight(family, "formation", cfg, scene)
                self.assertGreater(min(entry["distance_m"] for end in proof["endpoints"].values()
                                       for entry in end.values()), 1.7)
                old = {**cfg, "start": [4., 5.6, 0.], "goal": [4.+x, 5.6+y, heading]}
                with self.assertRaisesRegex(ValueError, "goal: inflated .* overlaps cnc_01"):
                    fixture_preflight(family, "formation", old, fixture_scene(old, "formation"))
                with TemporaryDirectory() as archive:
                    (Path(archive)/"resolved_config.yaml").write_text(json.dumps(old))
                    self.assertFalse(same_fixture(cfg, Path(archive)))

    def test_simulation_launch_uses_selected_spawn(self):
        path = ROOT/"ros2_ws/src/fwis_bringup/launch/simulation.launch.py"
        module_spec = importlib.util.spec_from_file_location("coptrans_fixture_spawn_launch", path)
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
        for task in ("single", "formation"):
            with self.subTest(task=task), TemporaryDirectory() as temporary, patch.dict(os.environ, ROS_LOG_DIR=temporary):
                work = Path(temporary)
                cfg = fixture_config("bend", task, "baseline")
                (work/"resolved_config.yaml").write_text(json.dumps(cfg))
                scene = fixture_scene(cfg, task)
                ctx = LaunchContext()
                ctx.launch_configurations.update(
                    scenario=cfg["scenario"], variant=cfg["scene_variant"], resolution="", task=task,
                    gui="false", rviz="false", body_twist="true", tracking_dashboard="false",
                    experiment_file=str(work/"resolved_config.yaml"), scene_directory=str(work/"scene"))
                # Assemble the normal launch graph; capture the robot creation arguments
                # at the Gazebo adapter and leave all resulting processes unstarted.
                with patch.object(module, "model_actions", return_value=([], "/probe", ("/probe", "/probe"))) as create:
                    module.setup(ctx)
                expected = {f"robot{i}": list(pose) for i, pose in scene.spawn_poses(task).items()}
                received = {call.args[2]: list(call.args[3]) for call in create.call_args_list}
                self.assertEqual(received, expected)
                meta = yaml.safe_load((work/"scene/scene_meta.yaml").read_text())
                self.assertEqual(meta["spawn_poses"], expected)
                if task == "formation":
                    sdf = ElementTree.parse(work/"scene/world.sdf")
                    pose = sdf.findtext(f"./world/model[@name='{scene.cargo.name}']/pose")
                    values = list(map(float, pose.split()))
                    self.assertEqual([values[0], values[1], values[5]], cfg["start"])


if __name__ == "__main__":
    unittest.main()
