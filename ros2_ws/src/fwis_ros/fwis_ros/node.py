"""ROS 2 parameters, QoS and time utilities used by CopTrans nodes."""

import math
import signal

from builtin_interfaces.msg import Duration, Time
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.wait_for_message import wait_for_message


_MISSING = object()
_DURABLE_TOPICS = {
    "/plan/seed", "/plan/seed_candidates", "/plan/search_status",
    "/plan/timed_trajectory", "/plan/quality_report",
    "/formation/job_state", "/formation/optimizer_status",
    "/formation/quality_report", "/formation/plan_seed",
    "/formation/playback_plan", "/formation/timed_trajectory",
    "/formation/planning_debug", "/formation/status",
}


class PlannerROSNode(Node):
    def __init__(self, name, *, live_clock=False):
        super().__init__(name, automatically_declare_parameters_from_overrides=True)
        # Keep planner callbacks serialized while the default group takes
        # /clock updates during a long optimization callback.
        self.algorithm_callbacks = MutuallyExclusiveCallbackGroup() if live_clock else None

    def param(self, name, default=_MISSING):
        name = name.lstrip("~").replace("/", ".")
        if self.has_parameter(name):
            return self.get_parameter(name).value
        children = self.get_parameters_by_prefix(name)
        if children:
            result = {}
            for key, parameter in children.items():
                parent = result
                parts = key.split(".")
                for part in parts[:-1]:
                    parent = parent.setdefault(part, {})
                parent[parts[-1]] = parameter.value
            return result
        if default is _MISSING:
            raise KeyError(name)
        if default is not None and not isinstance(default, (dict, list)):
            self.declare_parameter(name, default)
        return default

    def _input_qos(self, topic, depth):
        durable = (self.param("map_transient_local", True) if topic == "/map"
                   else topic in _DURABLE_TOPICS)
        return _qos(depth, durable)

    def publisher(self, topic, msg_type, *, queue_size, latch=False):
        return self.create_publisher(msg_type, topic, _qos(queue_size, latch))

    def subscribe(self, topic, msg_type, callback, *, queue_size):
        return self.create_subscription(
            msg_type, topic, callback, self._input_qos(topic, queue_size),
            callback_group=self.algorithm_callbacks)

    def receive(self, topic, msg_type, *, timeout):
        received, message = wait_for_message(
            msg_type, self, topic, qos_profile=self._input_qos(topic, 1),
            time_to_wait=timeout)
        if not received:
            raise TimeoutError(topic)
        return message

    def warn_throttle(self, period, message, *args):
        self.get_logger().warning(
            message % args if args else message, throttle_duration_sec=float(period))

    def stamp(self):
        return self.get_clock().now().to_msg()

    def log(self, level, message, *args):
        text = message % args if args else str(message)
        logger = self.get_logger()
        if level == "info":
            logger.info(text)
        elif level == "warning":
            logger.warning(text)
        elif level == "debug":
            logger.debug(text)
        elif level == "error":
            logger.error(text)
        else:
            logger.fatal(text)


def _qos(depth, durable):
    return QoSProfile(
        depth=depth, durability=(DurabilityPolicy.TRANSIENT_LOCAL if durable
                                 else DurabilityPolicy.VOLATILE))


def run_node(name, component, *, threads=1, live_clock=False):
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    # Finish taking the current message before tearing down the ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.SIGTERM)
    previous_sigint = signal.signal(signal.SIGINT, stop)
    node = PlannerROSNode(name, live_clock=live_clock)
    if live_clock:
        threads = max(2, threads)
    executor = MultiThreadedExecutor(num_threads=threads) if threads > 1 else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        component(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)


def time_message(sec=0, nanosec=0):
    return Time(sec=int(sec), nanosec=int(nanosec))


def duration_message(sec=0, nanosec=0):
    return Duration(sec=int(sec), nanosec=int(nanosec))


def duration_from_seconds(value):
    sec, nanosec = divmod(int(float(value) * 1_000_000_000), 1_000_000_000)
    return duration_message(sec, nanosec)


def seconds(message):
    return message.sec + message.nanosec / 1_000_000_000.0


def add_duration(stamp, duration):
    carry, nanosec = divmod(stamp.nanosec + duration.nanosec, 1_000_000_000)
    return time_message(stamp.sec + duration.sec + carry, nanosec)


def elapsed(now, then):
    return (now.sec - then.sec) + (now.nanosec - then.nanosec) / 1_000_000_000.0


def quaternion_from_euler(roll, pitch, yaw):
    roll, pitch, yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
