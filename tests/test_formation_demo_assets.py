"""S10 demo launch, RViz and playback-install asset tests (no ROS)."""
from pathlib import Path
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


class FormationLaunchTest(unittest.TestCase):

    def _nodes(self):
        root = ElementTree.parse(LAUNCH).getroot()
        return root.findall("node")

    def test_launch_has_exactly_one_of_each_formation_process(self):
        nodes = self._nodes()
        names = [node.get("name") for node in nodes]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(names.count("fwis_map_publisher"), 1)
        self.assertEqual(names.count("formation_planner"), 1)
        self.assertEqual(names.count("formation_optimizer"), 1)
        self.assertEqual(names.count("formation_playback"), 1)
        types = {node.get("name"): node.get("type") for node in nodes}
        self.assertEqual(
            types["formation_planner"], "formation_planner_node")
        self.assertEqual(
            types["formation_optimizer"], "formation_optimizer_node.py")
        self.assertEqual(
            types["formation_playback"], "formation_playback.py")

    def test_launch_never_starts_single_vehicle_nodes_or_extra_planners(self):
        nodes = self._nodes()
        forbidden_types = {
            "optimizer_node.py", "playback.py", "collect_node.py",
            "formation_planner_node.py",
        }
        for node in nodes:
            self.assertNotIn(node.get("type"), forbidden_types)
        planner_types = [
            node for node in nodes
            if "planner" in (node.get("type") or "")
        ]
        self.assertEqual(len(planner_types), 1)
        self.assertEqual(planner_types[0].get("type"),
                         "formation_planner_node")

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
