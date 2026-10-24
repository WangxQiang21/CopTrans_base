#!/usr/bin/env python3
"""Independently check saved v2 corridor records against actual Marker points."""
import argparse
import json
import math
from pathlib import Path


def close(a, b, tol=1e-7):
    return len(a) == len(b) and all(abs(x-y) <= tol for x,y in zip(a,b))


def audit(probe):
    report = probe["quality_report"]
    corridor = report["corridor"]
    footprints = corridor["footprints"]
    records = corridor["records"]
    errors = []
    expected = {}
    def require(condition, reason):
        if not condition:
            errors.append(reason)

    def add(namespace, corners):
        expected.setdefault(namespace, []).append(corners)

    display = probe["corridor_display"]
    visualization = report.get("visualization", {})
    indices = visualization.get("station_indices")
    if indices is None:
        # Backward compatibility for probes created before adaptive display
        # sampling recorded its exact station list.
        stride = max(1, len(footprints)//18)
        indices = list(range(0, len(footprints), stride))
        if indices[-1] != len(footprints)-1:
            indices.append(len(footprints)-1)
    require(len(indices) == len(set(indices)), "duplicate_display_station")
    require(all(isinstance(i, int) and 0 <= i < len(footprints)
                for i in indices), "invalid_display_station")
    require(visualization.get("displayed_stations", len(indices)) == len(indices),
            "displayed_station_count")
    signature = corridor["map_signature"]
    length, width = [2*x for x in signature["half_vehicle"]]
    physical_length, physical_width = corridor.get("physical_vehicle_size_m", [.738,.500])
    for footprint in footprints:
        p = footprint["reference_pose"]
        for key, l, w in (("body_corners_world_m", physical_length, physical_width),
                          ("margin_corners_world_m", length, width)):
            corners = footprint[key]
            a,b,c,_ = corners
            require(abs(math.dist(a,b)-l) < 1e-7 and abs(math.dist(b,c)-w) < 1e-7,
                    "footprint_dimensions")
            require(close([sum(q[0] for q in corners)/4, sum(q[1] for q in corners)/4],
                          [p["x"],p["y"]]), "footprint_center")
            heading = math.atan2(b[1]-a[1], b[0]-a[0])
            require(abs(math.atan2(math.sin(heading-p["th"]),
                                   math.cos(heading-p["th"]))) < 1e-7,
                    "footprint_body_yaw")
        if footprint["station"] in indices:
            add("v3_vehicle_footprints", footprint["body_corners_world_m"])
            add("v3_vehicle_margin_footprints", footprint["margin_corners_world_m"])
    for record in records:
        require(record["plan_id"] == report["plan_id"], "record_plan_id")
        require(record["map_hash"] == corridor["map_hash"], "record_map_hash")
        require(record["frame_id"] == "map", "record_frame")
        require(not record["used_in_optimization"], "unexpected_optimization_claim")
        p = record["reference_pose"]
        require(p == footprints[record["station"]]["reference_pose"], "station_reference")
        ox,oy = record["feature_offset"]
        x,y,hl,hw,yaw = record["box"]
        require(close([x,y], [p["x"]+ox*math.cos(p["th"])-oy*math.sin(p["th"]),
                              p["y"]+ox*math.sin(p["th"])+oy*math.cos(p["th"])]),
                "feature_center")
        require(abs(yaw-p["th"]) < 1e-7, "region_reference_yaw")
        corners = record["corners_world_m"]
        for px,py in corners:
            u = (px-x)*math.cos(yaw)+(py-y)*math.sin(yaw)
            v = -(px-x)*math.sin(yaw)+(py-y)*math.cos(yaw)
            require(abs(abs(u)-hl) < 1e-7 and abs(abs(v)-hw) < 1e-7,
                    "region_corner_transform")
        if (record["station"] in indices and
                corridor.get("feature_center_regions_visible",True)):
            namespace = ("v3_feature_center_disc_regions" if
                         record["certification"] == "disc_clearance_raster_guard" else
                         "v3_feature_center_point_or_uncertified_regions")
            add(namespace, corners)
    counts = {}
    for marker in probe["corridor_display"]["markers_world"]:
        require(marker["frame_id"] == "map", "marker_frame")
        if "trajectory_header" in probe:
            require(marker["stamp_s"] == probe["trajectory_header"]["stamp_s"],
                    "stale_marker_stamp")
        require(close(marker["pose_orientation_xyzw"], [0,0,0,1]), "double_rotation")
        require(close(marker["pose_position"], [0,0,0]), "double_translation")
        points = marker["points"]
        require(len(points)%8 == 0, "incomplete_polygon")
        namespace = marker["namespace"]
        suffix = "_plan_%d" % int(report["plan_id"])
        if namespace.endswith(suffix):
            namespace = namespace[:-len(suffix)]
        else:
            require(not namespace.endswith("_plan_"), "marker_plan_namespace")
        for k in range(0,len(points),8):
            polygon = [points[k+j][:2] for j in (0,1,3,5)]
            matches = expected.get(namespace, [])
            index = next((i for i,corners in enumerate(matches) if
                          all(close(a,b) for a,b in zip(polygon,corners))), None)
            require(index is not None, "marker_reference_mismatch:"+namespace)
            if index is not None:
                matches.pop(index)
            counts[namespace] = counts.get(namespace,0)+1
    require(not any(expected.values()), "missing_station_or_feature_marker")
    return {"pass_check": not errors, "errors": errors,
            "stations_checked": len(footprints), "features_checked": len(records),
            "actual_marker_polygon_counts": counts,
            "certification_counts": {kind: sum(r["certification"] == kind for r in records)
                                     for kind in sorted({r["certification"] for r in records})}}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("probe", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(json.loads(args.probe.read_text()))
    encoded = json.dumps(result, indent=2)+"\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end="")
    raise SystemExit(0 if result["pass_check"] else 1)
