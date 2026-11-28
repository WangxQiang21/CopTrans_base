"""Build the existing Ranger RViz layout for each supported formation."""

from pathlib import Path
import tempfile

import yaml


def rviz_config(description, directory, scene, task, *, tracking_dashboard=False, show_corridor=False,
                show_cargo_trace=True, interactive=False, assembly=False, experiment_id="",
                config_path=None):
    vehicles = scene.spawn_poses(task)
    count = len(vehicles)
    blade = task == "blade_pair"
    model_template = 3 if blade else count
    data = yaml.safe_load((description / "rviz" / f"ranger_{model_template}.rviz").read_text())
    manager = data["Visualization Manager"]
    manager["Global Options"]["Fixed Frame"] = "map"
    manager["Displays"] = [display for display in manager["Displays"]
                           if display["Class"] != "rviz_default_plugins/Grid"]
    if blade:
        manager["Displays"] = [display for display in manager["Displays"]
                               if display.get("TF Prefix") in (None, *(f"robot{i}" for i in vehicles))]
    qos = {"Durability Policy": "Transient Local", "Reliability Policy": "Reliable",
           "History Policy": "Keep Last", "Depth": 1}
    manager["Displays"].insert(0, {
        "Class": "rviz_default_plugins/Map", "Name": "Planning map", "Enabled": True,
        "Topic": {"Value": "/map", **qos}, "Alpha": .80,
    })
    if task == "formation" or blade:
        manager["Displays"].append({
            "Class": "rviz_default_plugins/MarkerArray", "Name": "Cargo and centre mounts",
            "Enabled": True, "Topic": {"Value": "/scene/markers", **qos},
        })
        if show_cargo_trace:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/MarkerArray", "Name": "货物包络轨迹",
                "Enabled": True, "Topic": {"Value": "/formation/cargo_footprint_trace", **qos},
            })
        colors = ("36; 127; 176", "198; 103; 33", "103; 74; 153")
        paths = [(f"Vehicle {index} plan", f"/formation/path/vehicle_{index}", color)
                 for index, color in zip(scene.vehicle_ids, colors)]
        if assembly:
            paths += [(f"Vehicle {index} assembly", f"/formation/assembly_path/vehicle_{index}", color)
                      for index, color in zip(scene.vehicle_ids, colors)]
        if show_corridor:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/MarkerArray", "Name": "Convex corridor guide",
                "Enabled": True, "Topic": {"Value": "/formation/convex_corridor_guide", **qos},
            })
        if interactive or blade:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/MarkerArray", "Name": "编队起终点",
                "Enabled": True, "Topic": {"Value": "/formation/interactive_endpoints", **qos},
            })
        if interactive:
            tool_qos = {**qos, "Durability Policy": "Volatile"}
            manager.setdefault("Tools", []).extend([
                {"Class": "rviz_default_plugins/SetInitialPose", "Topic": {"Value": "/initialpose", **tool_qos}},
                {"Class": "rviz_default_plugins/SetGoal", "Topic": {"Value": "/move_base_simple/goal", **tool_qos}},
            ])
        if interactive or assembly or blade:
            data.setdefault("Panels", []).append({"Class": "fwis_rviz_plugins/PlanningPanel", "Name": "规划与执行"})
    else:
        paths = [("Single-vehicle plan", "/plan/path", "36; 127; 176")]
    for name, topic, color in paths:
        manager["Displays"].append({
            "Class": "rviz_default_plugins/Path", "Name": name, "Enabled": True,
            "Topic": {"Value": topic, **qos}, "Color": color, "Alpha": 1.0,
            "Line Style": "Billboards", "Line Width": .055,
            "Offset": {"X": 0, "Y": 0, "Z": .39},
        })
    if tracking_dashboard:
        for index in vehicles:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/Axes", "Name": f"Vehicle {index} control frame",
                "Enabled": True, "Reference Frame": f"robot{index}/base_link",
                "Length": .43, "Radius": .014,
            })
        dashboard = {
            "Class": "fwis_rviz_plugins/TrackingHUD", "Name": "Tracking dashboard",
            "Enabled": True, "Show Cargo Marker": False, "Experiment ID": experiment_id,
        }
        if blade:
            dashboard["Vehicle Count"] = count
        manager["Displays"].append(dashboard)
    w, h = scene.size
    x, y, _yaw = scene.experiments[task]["start"]
    manager["Views"] = {
        "Current": {"Class": "rviz_default_plugins/TopDownOrtho", "X": w / 2, "Y": h / 2,
                    "Scale": min(650 / w, 590 / h)},
        "Saved": [{"Class": "rviz_default_plugins/Orbit", "Name": "Loading bay",
                   "Target Frame": "<Fixed Frame>", "Yaw": 3.45, "Pitch": .62, "Distance": 5.3,
                   "Focal Point": {"X": x + .5, "Y": y, "Z": .33}}],
    }
    data["Window Geometry"] = {"Width": 1190, "Height": 805}
    path = config_path if config_path is not None else directory / "scene.rviz"
    with tempfile.NamedTemporaryFile("w", dir=path.parent, encoding="utf-8", delete=False) as out:
        yaml.safe_dump(data, out, sort_keys=False)
    Path(out.name).replace(path)
    return path
