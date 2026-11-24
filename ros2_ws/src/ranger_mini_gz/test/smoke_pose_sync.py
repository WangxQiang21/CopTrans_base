#!/usr/bin/env python3
"""Move the simulated models independently and check the TF used by RViz."""

import argparse
import math
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=(1, 3), required=True)
    args = parser.parse_args()
    names = [f"robot{index}" for index in range(1, args.robots + 1)]
    rclpy.init()
    node = rclpy.create_node("ranger_pose_smoke")
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    clocks = []

    def on_clock(message):
        stamp = message.clock
        clocks.append(stamp.sec * 1_000_000_000 + stamp.nanosec)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)

    def lookup_updates(expected, yaw):
        deadline = time.monotonic() + 20
        observed = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                transforms = {name: buffer.lookup_transform("world", f"{name}/base_link", Time())
                              for name in names}
                for name in names:
                    buffer.lookup_transform(f"{name}/base_link", f"{name}/fl_wheel_link", Time())
            except TransformException:
                continue
            for name, transform in transforms.items():
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                observed[name] = (translation.x, translation.y, translation.z,
                                  2.0 * math.atan2(rotation.z, rotation.w))
            if (len(clocks) >= 2 and clocks[-1] > clocks[0] and
                    all(math.dist(observed[name][:3], expected[name]) < 1e-6
                        and abs(observed[name][3] - yaw[name]) < 1e-6 for name in names)):
                return observed
        raise AssertionError(f"transform timeout: observed={observed}, expected={expected}, yaw={yaw}")

    try:
        expected = {name: (0., index * 1.5, .322) for index, name in enumerate(names)}
        yaw = dict.fromkeys(names, 0.0)
        print(f"initial: {lookup_updates(expected, yaw)}", flush=True)
        for index, name in enumerate(names):
            _, y, z = expected[name]
            expected[name] = (1.05 + index * .3, y - .3, z)
            yaw[name] = .15 * (index + 1)
            x, y, z = expected[name]
            qz, qw = math.sin(yaw[name] * .5), math.cos(yaw[name] * .5)
            request = (f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
                       f'orientation {{z: {qz} w: {qw}}}')
            result = subprocess.run(
                ["gz", "service", "-s", "/world/whut/set_pose/blocking",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000", "--req", request],
                capture_output=True, text=True, check=True, timeout=7)
            assert "data: true" in result.stdout, result.stdout + result.stderr
            print(f"after moving {name}: {lookup_updates(expected, yaw)}", flush=True)
        print(f"/clock advanced by {(clocks[-1] - clocks[0]) * 1e-9:.3f} s", flush=True)
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
