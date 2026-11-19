"""S10 demo launch, RViz and playback-install asset tests (no ROS)."""
from pathlib import Path
import math
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
        self.assertEqual(args["backend"], "virtual_rigid_a")
        self.assertEqual(args["virtual_a_guidance_mode"], "corridor")
        self.assertEqual(args["virtual_a_corridor_enforcement"], "hard")
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

    def test_launch_passes_research_config_only_to_the_virtual_node(self):
        root = ElementTree.parse(LAUNCH).getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        planner = nodes["formation_uniform_planner"]
        params = {}
        for param in planner.findall("param"):
            params[param.get("name")] = (
                param.get("value"), param.get("if") or "")
        self.assertEqual(params["virtual_a_guidance_mode"][0],
                         "$(arg virtual_a_guidance_mode)")
        self.assertEqual(params["virtual_a_corridor_enforcement"][0],
                         "$(arg virtual_a_corridor_enforcement)")
        for name in ("virtual_a_guidance_mode",
                     "virtual_a_corridor_enforcement"):
            self.assertIn("arg('backend') == 'virtual_rigid_a'",
                          params[name][1])
        for legacy_name in ("formation_planner", "formation_optimizer"):
            legacy_params = {
                param.get("name") for param in nodes[legacy_name].findall(
                    "param")}
            self.assertNotIn("virtual_a_guidance_mode", legacy_params)
            self.assertNotIn("virtual_a_corridor_enforcement", legacy_params)

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

    def test_node_main_reads_and_validates_the_research_entry(self):
        source = UNIFORM_PLANNER.read_text(encoding="utf-8")
        self.assertIn('"~virtual_a_guidance_mode", "corridor"', source)
        self.assertIn('"~virtual_a_corridor_enforcement", "hard"', source)
        self.assertIn("research_planner_config(", source)
        self.assertIn("planner_config.guidance_mode", source)
        self.assertIn("planner_config.corridor_enforcement", source)
        self.assertNotIn('"~virtual_a_guidance_mode", "legacy_route"',
                         source)

    def test_uniform_planner_script_is_installed_by_catkin(self):
        cmake = OPTIMIZER_CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_install_python", cmake)
        self.assertIn("scripts/formation_uniform_planner_node.py", cmake)
        self.assertTrue(UNIFORM_PLANNER.is_file())

    def test_launch_defaults_match_the_frozen_uniform_contract(self):
        """FR05-R1: the demo must not ship wider tolerances or budgets."""
        root = ElementTree.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["max_expansions"], "10000")
        self.assertAlmostEqual(float(args["uniform_max_seconds"]), 30.0,
                               places=12)
        self.assertAlmostEqual(float(args["uniform_goal_xy_tol"]), 0.02,
                               places=15)
        self.assertAlmostEqual(float(args["uniform_goal_psi_tol"]),
                               math.radians(1.0), places=15)
        node = next(node for node in root.findall("node")
                    if node.get("name") == "formation_uniform_planner")
        params = {param.get("name"): param.get("value")
                  for param in node.findall("param")}
        self.assertEqual(params["goal_xy_tol"], "$(arg uniform_goal_xy_tol)")
        self.assertEqual(params["goal_psi_tol"], "$(arg uniform_goal_psi_tol)")
        self.assertEqual(params["goal_heading_tol"],
                         "$(arg uniform_goal_psi_tol)")
        self.assertEqual(params["max_expansions"], "$(arg max_expansions)")
        self.assertEqual(params["max_seconds"], "$(arg uniform_max_seconds)")

    def test_uniform_planner_defaults_reference_frozen_config(self):
        """FR05-R1: node fallbacks are the FR03/FR04 defaults, not copies."""
        source = UNIFORM_PLANNER.read_text(encoding="utf-8")
        self.assertIn('"~goal_xy_tol", SuccessorConfig.goal_xy_tol_m',
                      source)
        self.assertIn('"~goal_psi_tol", SuccessorConfig.goal_psi_tol_rad',
                      source)
        self.assertIn(
            '"~goal_heading_tol", SuccessorConfig.goal_heading_tol_rad',
            source)
        self.assertIn(
            '"~max_expansions", GraphSearchConfig.max_expansions', source)
        self.assertIn('"~max_seconds", GraphSearchConfig.max_seconds', source)
        for stale in ('"~goal_xy_tol", 0.16', '"~goal_psi_tol", 0.16',
                      '"~goal_heading_tol", 0.16',
                      '"~max_expansions", 50000', '"~max_seconds", 20.0'):
            self.assertNotIn(stale, source)

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

    def test_launch_configures_the_fixed_corner_hud(self):
        root = ElementTree.parse(LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default")
                for arg in root.findall("arg")}
        self.assertEqual(args["hud"], "true")
        self.assertEqual(args["hud_left"], "900")
        self.assertEqual(args["hud_top"], "20")
        self.assertEqual(args["hud_width"], "360")
        self.assertEqual(args["hud_height"], "270")
        node = next(node for node in root.findall("node")
                    if node.get("name") == "formation_playback")
        params = {param.get("name"): param.get("value")
                  for param in node.findall("param")}
        for name, source in (
                ("hud_enabled", "hud"),
                ("hud_left", "hud_left"),
                ("hud_top", "hud_top"),
                ("hud_width", "hud_width"),
                ("hud_height", "hud_height")):
            self.assertEqual(params[name], "$(arg {})".format(source))
        self.assertEqual(params["show_status_text"], "false")
        self.assertEqual(params["show_cargo_reason"], "false")

    def test_playback_package_declares_the_hud_runtime_dependency(self):
        package = ElementTree.parse(
            ROOT / "catkin_ws/src/fwis_v3_playback/package.xml").getroot()
        dependencies = {
            element.text for element in package.findall("exec_depend")}
        self.assertIn("jsk_rviz_plugins", dependencies)

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

    def test_default_backend_is_the_virtual_research_entry(self):
        result = self._run("--map", "map04")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("backend:=virtual_rigid_a", result.stdout)
        self.assertIn("virtual_a_guidance_mode:=corridor", result.stdout)
        self.assertIn("virtual_a_corridor_enforcement:=hard", result.stdout)
        self.assertIn("auto_goal:=false", result.stdout)
        self.assertNotIn("backend:=uniform_v02", result.stdout)
        self.assertNotIn("backend:=legacy", result.stdout)
        for name in ("backend:=", "virtual_a_guidance_mode:=",
                     "virtual_a_corridor_enforcement:=", "auto_goal:="):
            self.assertEqual(result.stdout.count(name), 1, name)

    def test_explicit_research_parameters_are_preserved(self):
        result = self._run(
            "--map", "map04",
            "virtual_a_guidance_mode:=legacy_route",
            "virtual_a_corridor_enforcement:=soft",
            "auto_goal:=true")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("virtual_a_guidance_mode:=legacy_route", result.stdout)
        self.assertIn("virtual_a_corridor_enforcement:=soft", result.stdout)
        self.assertIn("auto_goal:=true", result.stdout)
        self.assertNotIn("virtual_a_guidance_mode:=corridor", result.stdout)
        self.assertNotIn("virtual_a_corridor_enforcement:=hard", result.stdout)
        self.assertNotIn("auto_goal:=false", result.stdout)

    def test_duplicate_research_parameters_are_rejected(self):
        cases = (
            ("virtual_a_guidance_mode:=corridor",
             "virtual_a_guidance_mode:=corridor",
             "virtual_a_guidance_mode:= 只能指定一次"),
            ("virtual_a_corridor_enforcement:=hard",
             "virtual_a_corridor_enforcement:=soft",
             "virtual_a_corridor_enforcement:= 只能指定一次"),
            ("auto_goal:=false", "auto_goal:=false",
             "auto_goal:= 只能指定一次"),
        )
        for first, second, message in cases:
            result = self._run("--map", "map04", first, second)
            self.assertEqual(result.returncode, 2, result.stdout)
            self.assertIn(message, result.stderr)
            self.assertNotIn("formation_demo.launch", result.stdout)

    def test_explicit_uniform_or_legacy_keeps_historical_auto_goal(self):
        for backend in ("uniform_v02", "legacy"):
            result = self._run("--map", "map04", "--backend", backend)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertNotIn("virtual_a_guidance_mode:=", result.stdout)
            self.assertNotIn("virtual_a_corridor_enforcement:=", result.stdout)
            # map04 has scene_meta, so the historical manual-map policy still
            # enables the preset goal for both old backends.
            self.assertIn("auto_goal:=true", result.stdout)

    def test_cli_and_launch_arg_forms_select_the_same_backend(self):
        for args in (("--backend", "legacy"), ("backend:=legacy",)):
            result = self._run("--map", "map04", *args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("backend:=legacy", result.stdout)
            self.assertEqual(result.stdout.count("backend:="), 1)
            self.assertNotIn("virtual_a_guidance_mode:=", result.stdout)
            self.assertNotIn("virtual_a_corridor_enforcement:=", result.stdout)

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


class RunVirtualAResearchEntryTest(unittest.TestCase):

    def _run(self, *args):
        return subprocess.run(
            ["bash", str(ROOT / "run_virtual_a.sh"), "--dry-run", *args],
            cwd=ROOT, capture_output=True, text=True)

    def test_wrapper_fills_only_missing_research_parameters(self):
        result = self._run("--map", "map04")
        self.assertEqual(result.returncode, 0, result.stderr)
        for expected in ("backend:=virtual_rigid_a",
                         "virtual_a_guidance_mode:=corridor",
                         "virtual_a_corridor_enforcement:=hard",
                         "auto_goal:=false"):
            self.assertIn(expected, result.stdout)
            self.assertEqual(result.stdout.count(expected), 1, expected)

    def test_wrapper_preserves_explicit_overrides(self):
        result = self._run(
            "--map", "map04",
            "virtual_a_guidance_mode:=legacy_route",
            "virtual_a_corridor_enforcement:=soft",
            "auto_goal:=true")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("virtual_a_guidance_mode:=legacy_route", result.stdout)
        self.assertIn("virtual_a_corridor_enforcement:=soft", result.stdout)
        self.assertIn("auto_goal:=true", result.stdout)
        self.assertNotIn("virtual_a_guidance_mode:=corridor", result.stdout)
        self.assertNotIn("virtual_a_corridor_enforcement:=hard", result.stdout)


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

    def test_demo_rviz_has_the_hud_and_corridor_layers(self):
        displays = self.document["Visualization Manager"]["Displays"]
        hud = next(item for item in displays
                   if item.get("Class") == "jsk_rviz_plugin/OverlayText")
        self.assertEqual(hud["Topic"], "/formation/hud")
        self.assertEqual(hud["left"], 900)
        self.assertEqual(hud["top"], 20)
        self.assertEqual(hud["width"], 360)
        self.assertEqual(hud["height"], 270)
        self.assertTrue(hud["Overtake Position Properties"])
        corridor = next(
            item for item in displays
            if item.get("Class") == "rviz/MarkerArray"
            and item.get("Marker Topic") == "/formation/corridors")
        self.assertTrue(corridor["Enabled"])

    def test_all_path_layers_use_adjustable_rviz_path_displays(self):
        displays = self.document["Visualization Manager"]["Displays"]
        paths = [item for item in displays
                 if item.get("Class") == "rviz/Path"]
        expected_topics = {
            "/formation/path/vehicle_{}".format(vehicle_id)
            for vehicle_id in (1, 2, 3)}
        expected_topics.add("/formation/path/f_candidate")
        expected_topics.update(
            "/formation/path/candidate_vehicle_{}".format(vehicle_id)
            for vehicle_id in (1, 2, 3))
        expected_topics.update(
            "/formation/path/astar_raw_{}".format(route_id)
            for route_id in (0, 1, 2))
        expected_topics.update(
            "/formation/path/portal_guide_{}".format(route_id)
            for route_id in (0, 1, 2))
        self.assertEqual({item["Topic"] for item in paths}, expected_topics)
        for item in paths:
            self.assertTrue(item["Enabled"])
            self.assertIn("Color", item)
            self.assertIn("Line Width", item)
            self.assertGreater(item["Line Width"], 0.0)
        marker_display = next(
            item for item in displays
            if item.get("Class") == "rviz/MarkerArray"
            and item.get("Marker Topic") == "/formation/markers")
        # The marker layer still carries the OBB, heading, history and
        # labels; the full path itself is an adjustable Path display.
        self.assertTrue(marker_display["Enabled"])
        guide_markers = next(
            item for item in displays
            if item.get("Class") == "rviz/MarkerArray"
            and item.get("Marker Topic") == "/formation/guides")
        self.assertEqual(guide_markers["Name"], "Guide labels (markers)")
        candidate_markers = next(
            item for item in displays
            if item.get("Class") == "rviz/MarkerArray"
            and item.get("Marker Topic") == "/formation/planning_candidates")
        self.assertEqual(candidate_markers["Name"],
                         "Candidate OBBs and labels (markers)")

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
