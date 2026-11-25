#!/usr/bin/env python3
"""Validate native ROS qualification runs and project raw measurements to the V3 gate."""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path


def write(path, document):
    payload=json.dumps(document, indent=2, sort_keys=True, allow_nan=False)+"\n"
    tmp=path.with_suffix(".tmp")
    tmp.write_text(payload)
    tmp.replace(path)


def rows(path, count, profile):
    records=json.loads(path.read_text())
    if not records or any(r["task"]!=count or r["profile"]!=profile for r in records):
        raise ValueError(f"Missing rows or incorrect setup/profile in {path}")
    return records


def main():
    p=argparse.ArgumentParser()
    p.add_argument("run",type=Path)
    p.add_argument("output",type=Path)
    p.add_argument("--parity", required=True, type=Path)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    before=(args.run/"binaries.before.sha256").read_bytes()
    if not before or before!=(args.run/"binaries.after.sha256").read_bytes():
        raise ValueError("native process binaries changed during qualification")
    delays={}
    faults={}
    groups={}
    raw={}
    for count, task in ((1,"single"),(3,"formation")):
        for profile in ("baseline","standard"):
            key=f"{task}/{profile}"
            path=args.run/f"{count}_{profile}_latency.json"
            attempts=rows(path,count,profile)
            raw[path.name]=sha256(path.read_bytes()).hexdigest()
            if (len(attempts)!=35 or any(r["id"]!=i or r["warmup"]!=(i<=5) or
                   r["fault"]!="admission_only" for i,r in enumerate(attempts,1))):
                raise ValueError(f"Expected five warmups and thirty attempts for {key}")
            delays[key]=[r["delivery_and_admission_wall_s"] for r in attempts if not r["warmup"]]
            if any(not math.isfinite(x) or x<=0 for x in delays[key]):
                raise ValueError(f"Invalid delivery timing at {key}")
            seen={}
            for suite in ("faults","expired","rollback","repeat_ack","ack_before_envelope","clock_delivery_order","clock_not_synchronized","forged_source_epoch"):
                path=args.run/f"{count}_{profile}_{suite}.json"
                cases=rows(path,count,profile)
                raw[path.name]=sha256(path.read_bytes()).hexdigest()
                for r in cases:
                    if r["fault"] in seen:
                        raise ValueError(f"Duplicate fault {r['fault']} at {key}")
                    seen[r["fault"]]=r
            success={"success","repeat_ack","ack_before_envelope","clock_delivery_order"}
            stopped={"tamper","expired","missing_member","stale_feedback","wrong_mode","map_replaced","permit_loss",
                     "wrong_epoch","future_permit","revoked_permit","nan_feedback","clock_rollback","clock_not_synchronized","forged_source_epoch"}
            if set(seen)!=success|stopped or any(seen[k]["phase"]!="COMPLETE" for k in success):
                raise ValueError(f"Missing successful controls or fault cases at {key}")
            if any(seen[k]["phase"]!="STOPPED" for k in stopped):
                raise ValueError(f"Motion permitted on fault at {key}")
            if any(r["stale_feedback_moving_commands"]!=0 or r["nonfinite_commands"]!=0 for r in seen.values()):
                raise ValueError(f"Invalid output at {key}")
            if seen["missing_member"]["release_s"] is not None or seen["expired"]["release_s"] is not None:
                raise ValueError(f"Released without fresh plan and all members at {key}")
            if seen["expired"]["reason"]!="invalid future epoch" or seen["clock_rollback"]["reason"]!="clock_rollback":
                raise ValueError(f"Clock contract was not exercised at {key}")
            if (seen["clock_delivery_order"]["publication_clock_gap_s"]<=.1 or
                    seen["clock_not_synchronized"]["release_s"] is not None or
                    seen["clock_not_synchronized"]["reason"]!="publication_clock_not_synchronized" or
                    seen["forged_source_epoch"]["release_s"] is not None or
                    seen["forged_source_epoch"]["reason"]!="invalid future epoch"):
                raise ValueError(f"Missing independent publication/receiver clock checks at {key}")
            for name in ("wrong_epoch","future_permit","revoked_permit","permit_loss"):
                if "formation_permit_missing_or_revoked" not in seen[name]["tracker_reasons"]:
                    raise ValueError(f"{key} did not exercise permit rejection on {name}")
            if "missing_stale_or_invalid_feedback" not in seen["nan_feedback"]["tracker_reasons"]:
                raise ValueError(f"{key} did not exercise nonfinite feedback")
            if profile=="standard": groups[task]=list(seen.values())
            faults[key]={name:{"outcome":r["phase"],"reason":r["reason"],"tracker_reasons":r["tracker_reasons"],
                               "publication_clock_gap_s":r.get("publication_clock_gap_s",0.)}
                         for name,r in seen.items()}
    parity={}
    for algo in ("nonlinear","pid"):
        path=args.parity/f"parity_{algo}.json"
        result=json.loads(path.read_text())
        if (result["ticks"]<=0 or result["matched_fsm_ticks"]!=result["ticks"] or
                result["relative_tolerance"]!=0. or result["nonfinite_outputs"]!=0 or
                result["max_absolute_error"]>1e-6):
            raise ValueError(f"Controller parity failed for {algo}")
        parity[algo]=result
        raw[path.name]=sha256(path.read_bytes()).hexdigest()
    write(args.output/"delivery.json",delays)
    write(args.output/"faults.json",dict(old_epoch_rejected=True,clock_rollback_stopped=True,late_permit_stopped=True,
          invalid_numeric_commands=0,scenarios=faults,controller_parity=parity,
          binary_manifest_sha256=sha256(before).hexdigest(),raw_file_hashes=raw,raw_directory=str(args.run.resolve())))
    for task, examples in groups.items(): write(args.output/(task+".json"),examples)


if __name__=="__main__":
    main()
