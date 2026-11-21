#!/usr/bin/env python3
"""Frame the scaled map in RViz without changing the persisted template."""
from pathlib import Path
import sys
from PIL import Image
import yaml


def configure(map_dir, template, output, collection=False):
    map_dir = Path(map_dir)
    metadata = yaml.safe_load((map_dir / "map.yaml").read_text())
    with Image.open(map_dir / metadata["image"]) as image:
        width, height = image.size
    width *= metadata["resolution"]
    height *= metadata["resolution"]
    document = yaml.safe_load(Path(template).read_text())
    manager = document["Visualization Manager"]
    view = manager["Views"]["Current"]
    view["X"] = metadata["origin"][0] + width / 2
    view["Y"] = metadata["origin"][1] + height / 2
    view["Scale"] = min(900 / width, 600 / height)
    if collection:
        manager["Displays"].append({
            "Class": "rviz/Pose", "Name": "Saved test goal",
            "Enabled": True, "Value": True, "Topic": "/formation/case_saved_goal",
            "Shape": "Arrow", "Color": "16; 153; 68", "Alpha": 1,
            "Head Length": 0.45, "Head Radius": 0.17,
            "Shaft Length": 0.85, "Shaft Radius": 0.08,
        })
    Path(output).write_text(yaml.safe_dump(document, sort_keys=False))


if __name__ == "__main__":
    configure(*sys.argv[1:4], collection=sys.argv[4:] == ["--collect-cases"])
