"""S10 demo launch, RViz and playback-install asset tests (no ROS)."""
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ElementTree

import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch"
RVIZ = ROOT / "catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz"
PARAMS = ROOT / "catkin_ws/src/fwis_v3_demo/config/formation_params.yaml"
TRANSPORT_CONFIG = (ROOT / "catkin_ws/src/fwis_v3_demo/config/"
                    "transport_formation.yaml")
PLAYBACK_CMAKE = ROOT / "catkin_ws/src/fwis_v3_playback/CMakeLists.txt"
DEMO_CMAKE = ROOT / "catkin_ws/src/fwis_v3_demo/CMakeLists.txt"
OPTIMIZER_CMAKE = ROOT / "catkin_ws/src/fwis_v3_optimizer/CMakeLists.txt"
UNIFORM_PLANNER = (ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts/"
                   "formation_uniform_planner_node.py")


class FormationLaunchTest(unittest.TestCase):

    def _nodes(self):
        root = ElementTree.parse(LAUNCH).getroot()
        return root.findall("node")

    def test_launch_has_exactly_one_of_each_formation_process(self):
        nodes = self._nodes()
        names = [node.get("name") for node in nodes]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(names.count("fwis_map_publisher"), 1)
        self.assertEqual(names.count("formation_playback"), 1)
        types = {node.get("name"): node.get("type") for node in nodes}
        self.assertEqual(
            types["formation_playback"], "formation_playback.py")

    def test_launch_starts_one_backend_at_a_time(self):
        """Exactly two planner nodes exist and they can never co-start."""
        root = ElementTree.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["backend"], "uniform_v02")
        nodes = {node.get("name"): node for node in root.findall("node")}
        legacy = ("formation_planner", "formation_optimizer")
        uniform = "formation_uniform_planner"
        self.assertEqual(nodes[uniform].get("type"),
                         "formation_uniform_planner_node.py")
        for name in legacy:
            condition = nodes[name].get("if") or ""
            self.assertIn("arg('backend') == 'legacy'", condition)
        uniform_condition = nodes[uniform].get("if") or ""
        self.assertIn("arg('backend') == 'uniform_v02'", uniform_condition)
        # Reviewers: no node may publish trajectories in both backends.
        publishers = {name for name in nodes
                      if name in legacy or name == uniform}
        self.assertEqual(publishers, set(legacy) | {uniform})

    def test_launch_wires_backend_into_playback_and_bridge(self):
        root = ElementTree.parse(LAUNCH).getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        playback = nodes["formation_playback"]
        params = {param.get("name"): param.get("value")
                  for param in playback.findall("param")}
        self.assertEqual(params["backend"], "$(arg backend)")
        bridge = nodes["formation_goal_bridge"]
        params = {param.get("name"): param.get("value")
                  for param in bridge.findall("param")}
        self.assertEqual(params["admission_backend"], "$(arg backend)")
        self.assertIn("job_state_timeout_s", params)

    def test_launch_never_starts_single_vehicle_nodes_or_extra_planners(self):
        nodes = self._nodes()
        forbidden_types = {
            "optimizer_node.py", "playback.py", "collect_node.py",
        }
        for node in nodes:
            self.assertNotIn(node.get("type"), forbidden_types)
        planner_types = sorted(
            node.get("type") for node in nodes
            if "planner" in (node.get("type") or ""))
        self.assertEqual(planner_types,
                         ["formation_planner_node",
                          "formation_uniform_planner_node.py"])

    def test_uniform_planner_script_is_installed_by_catkin(self):
        cmake = OPTIMIZER_CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_install_python", cmake)
        self.assertIn("scripts/formation_uniform_planner_node.py", cmake)
        self.assertTrue(UNIFORM_PLANNER.is_file())

    def test_launch_uses_the_formation_params_and_rviz_files(self):
        root = ElementTree.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["rviz"], "false")
        self.assertEqual(args["playback_rate"], "25.0")
        self.assertIn("formation_params.yaml", args["config_file"])
        rviz_node = next(node for node in root.findall("node")
                         if node.get("name") == "rviz")
        self.assertEqual(rviz_node.get("args"), "-d $(arg rviz_config)")
        self.assertIn("formation_demo.rviz", args["rviz_config"])
        self.assertTrue(PARAMS.is_file())

    def test_playback_script_is_installed_by_catkin(self):
        cmake = PLAYBACK_CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_install_python", cmake)
        self.assertIn("scripts/formation_playback.py", cmake)
        self.assertIn("scripts/playback.py", cmake)

    def test_launch_starts_one_interactive_goal_bridge(self):
        root = ElementTree.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["goal_bridge"], "true")
        nodes = [node for node in root.findall("node")
                 if node.get("name") == "formation_goal_bridge"]
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.get("pkg"), "fwis_v3_demo")
        self.assertEqual(node.get("type"), "formation_goal_bridge.py")
        self.assertIsNone(node.find("rosparam[@param='offsets']"))
        loads = node.findall("rosparam")
        self.assertEqual(len(loads), 1)
        self.assertEqual(loads[0].get("command"), "load")
        self.assertEqual(loads[0].get("file"), "$(arg transport_config)")
        transport = yaml.safe_load(TRANSPORT_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(transport["offsets"],
                         [[1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        self.assertTrue(transport["goal_heading_free"])

    def test_goal_bridge_script_is_installed_by_catkin(self):
        cmake = DEMO_CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_install_python", cmake)
        self.assertIn("scripts/formation_goal_bridge.py", cmake)


class RunDemoBackendTest(unittest.TestCase):

    def _run(self, *args):
        return subprocess.run(
            ["bash", str(ROOT / "run_demo.sh"), "--dry-run", *args],
            cwd=ROOT, capture_output=True, text=True)

    def test_default_backend_is_uniform_v02(self):
        result = self._run("--map", "map04")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("backend:=uniform_v02", result.stdout)
        self.assertNotIn("backend:=legacy", result.stdout)

    def test_cli_and_launch_arg_forms_select_the_same_backend(self):
        for args in (("--backend", "legacy"), ("backend:=legacy",)):
            result = self._run("--map", "map04", *args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("backend:=legacy", result.stdout)
            self.assertEqual(result.stdout.count("backend:="), 1)

    def test_conflicting_or_unknown_backends_are_rejected(self):
        for args in (("--backend", "legacy", "backend:=uniform_v02"),
                     ("--backend", "nope"),
                     ("backend:=nope",)):
            result = self._run("--map", "map04", *args)
            self.assertEqual(result.returncode, 2, result.stdout)
            self.assertNotIn("formation_demo.launch", result.stdout)

    def test_backend_is_formation_only(self):
        result = self._run("--single", "--backend", "legacy")
        self.assertEqual(result.returncode, 2)
        self.assertIn("--backend 仅用于编队", result.stderr)


class FormationRvizTest(unittest.TestCase):

    def setUp(self):
        self.document = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))

    def test_map_and_formation_markers_are_displayed_in_map_frame(self):
        manager = self.document["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        displays = manager["Displays"]
        map_display = next(item for item in displays
                           if item.get("Name") == "Map")
        self.assertEqual(map_display["Topic"], "/map")
        marker_display = next(
            item for item in displays if item.get("Class") == "rviz/MarkerArray")
        self.assertEqual(marker_display["Marker Topic"], "/formation/markers")
        self.assertTrue(any(item.get("Class") == "rviz/TF"
                            for item in displays))

    def test_demo_rviz_does_not_reference_single_vehicle_topics(self):
        text = RVIZ.read_text(encoding="utf-8")
        for topic in ("/plan/markers", "/plan/mode_strip",
                      "/robot/heading_marker", "/plan/timed_trajectory"):
            self.assertNotIn(topic, text)

    def test_demo_rviz_exposes_goal_and_initial_pose_tools(self):
        manager = self.document["Visualization Manager"]
        tools = manager["Tools"]
        goal = next(item for item in tools
                    if item.get("Class") == "rviz/SetGoal")
        self.assertEqual(goal["Topic"], "/move_base_simple/goal")
        initial = next(item for item in tools
                       if item.get("Class") == "rviz/SetInitialPose")
        self.assertEqual(initial["Topic"], "/initialpose")


if __name__ == "__main__":
    unittest.main()
