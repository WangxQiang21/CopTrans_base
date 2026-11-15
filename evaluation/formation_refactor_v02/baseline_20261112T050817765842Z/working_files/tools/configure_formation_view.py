#!/usr/bin/env python3
"""Frame the scaled map in RViz without changing the persisted template."""
from pathlib import Path
import sys
from PIL import Image
import yaml


def configure(map_dir, template, output):
    map_dir = Path(map_dir)
    metadata = yaml.safe_load((map_dir / "map.yaml").read_text())
    with Image.open(map_dir / metadata["image"]) as image:
        width, height = image.size
    width *= metadata["resolution"]
    height *= metadata["resolution"]
    document = yaml.safe_load(Path(template).read_text())
    view = document["Visualization Manager"]["Views"]["Current"]
    view["X"] = metadata["origin"][0] + width / 2
    view["Y"] = metadata["origin"][1] + height / 2
    view["Scale"] = min(900 / width, 600 / height)
    Path(output).write_text(yaml.safe_dump(document, sort_keys=False))


if __name__ == "__main__":
    configure(*sys.argv[1:])
