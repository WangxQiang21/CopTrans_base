#!/usr/bin/env python3
"""Compare the recorded /formation/corridors messages with the exported C1 chain.

Read-only evidence check: it does not import the planner or rebuild geometry
with a different algorithm; the expected points are the fan/edge definitions
dictated by V1, evaluated on the vertices exported by corridor_display_fields.
"""
import json
import math
from pathlib import Path

RUN = Path(__file__).resolve().parent
FILL_Z = 0.005
EDGE_Z = 0.01
FILL_ALPHA = 0.15
EDGE_ALPHA = 0.95


def fan(vertices):
    first = vertices[0]
    points = []
    for index in range(1, len(vertices) - 1):
        points.extend((first, vertices[index], vertices[index + 1]))
    return [list(point) + [FILL_Z] for point in points]


def closed_edge(vertices):
    return [list(point) + [EDGE_Z] for point in vertices + [vertices[0]]]


def area(points):
    total = 0.0
    for index in range(0, len(points), 3):
        (x0, y0, _), (x1, y1, _), (x2, y2, _) = points[index:index + 3]
        total += abs((x1 - x0) * (y2 - y0) -
                     (x2 - x0) * (y1 - y0)) * 0.5
    return total


def compare(record, fixture, label):
    markers = record["markers"]
    display = fixture["corridors"][0]
    regions = display["regions"]
    result = {
        "fixture": label,
        "plan_id": fixture["plan_id"],
        "generation": fixture["generation"],
        "chain_digest": fixture["chain_digest"],
        "message_marker_count": len(markers),
        "first_action": markers[0]["action"],
        "regions": [],
    }
    assert markers[0]["action"] == "DELETEALL"
    assert len(markers) == 1 + 2 * len(regions)
    fills = [marker for marker in markers[1:]
             if marker["type"] == "TRIANGLE_LIST"]
    edges = [marker for marker in markers[1:]
             if marker["type"] == "LINE_STRIP"]
    assert len(fills) == len(edges) == len(regions)
    ids = {(marker["namespace"], marker["id"]) for marker in markers[1:]}
    assert len(ids) == len(markers) - 1
    for fill, edge, region in zip(fills, edges, regions):
        expected_fill = fan(region["vertices"])
        expected_edge = closed_edge(region["vertices"])
        assert fill["point_xyz"] == expected_fill
        assert edge["point_xyz"] == expected_edge
        assert fill["scale"] == [1.0, 1.0, 1.0]
        assert edge["scale"] == [0.03, 0.0, 0.0]
        assert abs(fill["color"][3] - FILL_ALPHA) < 1e-6
        assert abs(edge["color"][3] - EDGE_ALPHA) < 1e-6
        region_area = area(fill["point_xyz"])
        result["regions"].append({
            "region_id": region["region_id"],
            "vertices": len(region["vertices"]),
            "fill_points": len(fill["point_xyz"]),
            "edge_points": len(edge["point_xyz"]),
            "fill_scale": fill["scale"],
            "edge_scale": edge["scale"],
            "fill_z": fill["z_min"],
            "edge_z": edge["z_min"],
            "fill_alpha": fill["color"][3],
            "edge_alpha": edge["color"][3],
            "exported_area_m2": region["area_m2"],
            "marker_area_m2": region_area,
            "relative_area_error": abs(region_area - region["area_m2"]) /
            region["area_m2"],
        })
    return result


def main():
    records = [json.loads(line) for line in
               (RUN / "corridor_messages.jsonl").read_text().splitlines()]
    first = json.loads((RUN / "first_fixture.json").read_text())
    second = json.loads((RUN / "second_fixture.json").read_text())
    c1 = json.loads((RUN / "input_sample_chain.json").read_text())
    assert first["chain_digest"] == c1["chain"]["digest"]
    assert first["c1_sample_digest"] == c1["chain"]["digest"]
    assert second["regions"] == 1
    assert second["corridors"][0]["regions"][0]["vertices"] == \
        c1["chain"]["regions"][0]["vertices"]
    assert records[0]["markers"][0]["action"] == "DELETEALL"
    assert records[1]["marker_count"] == 5
    assert records[2]["marker_count"] == 3
    assert records[3]["marker_count"] == 1
    assert records[3]["markers"][0]["action"] == "DELETEALL"
    assert len(records) == 4, "stale plan and wrong generation must add none"
    report = {
        "c1_sample_digest": c1["chain"]["digest"],
        "c1_sample_grid_sha256": c1["chain"]["grid_sha256"],
        "recorded_messages": len(records),
        "messages": [
            {"index": 1, "markers": records[0]["marker_count"],
             "actions": [marker["action"] for marker in records[0]["markers"]]},
            compare(records[1], first, "full C1 chain"),
            compare(records[2], second, "first C1 region only"),
            {"index": 4, "markers": records[3]["marker_count"],
             "actions": [marker["action"] for marker in records[3]["markers"]]},
        ],
        "ignored_payloads": [
            {"case": "stale plan 91001 generation 1",
             "recorded_message_delta": 0},
            {"case": "wrong generation 99 for plan 91002",
             "recorded_message_delta": 0},
        ],
    }
    (RUN / "marker_verification.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()