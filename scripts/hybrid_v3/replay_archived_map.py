#!/usr/bin/env python3
"""Emit the normal native planner request for an archived ROS experiment."""
import argparse
from pathlib import Path
import sys

from cases import packed
from historical_routes import replay_request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--task", choices=["single", "formation"], required=True)
    parser.add_argument("--goal", type=float, nargs=3, help="explicit replacement goal in map x, y, yaw")
    parser.add_argument("--profile", choices=["standard", "baseline"], default="standard")
    args = parser.parse_args()
    payload = replay_request(args.archive, args.task, args.profile, goal=args.goal)
    sys.stdout.buffer.write(packed(payload) + b"\n")


if __name__ == "__main__":
    main()
