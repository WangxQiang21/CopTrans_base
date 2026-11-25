from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest


def runner(monkeypatch, *, interactive=False):
    source = Path(__file__).resolve().parents[1] / "scripts" / "run_experiment"
    loader = SourceFileLoader("coptrans_test_stop_runner", str(source))
    module = module_from_spec(spec_from_loader(loader.name, loader))
    loader.exec_module(module)
    clock = SimpleNamespace(wall=9., ros=7.)
    monkeypatch.setattr(module, "monotonic", lambda: clock.wall)

    class Recorder:
        on_command = module.RunExperiment.on_command
        on_odom = module.RunExperiment.on_odom
        finish = module.RunExperiment.finish
        velocity_observation = module.RunExperiment.velocity_observation
        termination_fields = module.RunExperiment.termination_fields

        def write_result(self):
            self.checkpoint = self.termination_fields(closing=False)

    r = Recorder()
    r.commands, r.odom, r.command_received_wall = {}, {}, {}
    r.traced_twist, r.moved = set(), set()
    r.plan_started, r.arm_stamp, r.plan_wall = 1., None, .2
    r.config, r.plan_keys = {"observation_s": 1.2}, {1: (1, 0)}
    r.result = r.end_wall = r.final_observation = r.native_cancel = None
    r.phase, r.interactive, r.solver_outcome, r.solver_elapsed_s = "running", interactive, "verified", .2
    r.now = lambda: clock.ros
    r.get_logger = lambda: SimpleNamespace(info=lambda *_: None)
    r.controller_log = SimpleNamespace(save_fault_tail=lambda *_: None)
    r.permit = lambda _: None
    r.trace = r.ui = lambda *a, **kw: None
    return r, clock


def sample(r, velocity, odom_sec):
    msg = Twist()
    msg.linear.x = velocity
    r.on_command(1, msg)
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = "map", "robot1/base_link"
    odom.header.stamp.sec = odom_sec
    odom.pose.pose.orientation.w = 1.
    odom.twist.twist.linear.x = velocity
    r.on_odom(1, odom)


@pytest.mark.parametrize("interactive", [False, True])
def test_fault_preserves_decision_and_observes_real_followup(monkeypatch, interactive):
    r, clock = runner(monkeypatch, interactive=interactive)
    sample(r, .0001343523815758571, 7)
    clock.wall = 10.
    r.finish("stopped", "benchmark_injected_cancel")
    assert r.observation_due_wall == pytest.approx(11.2)
    assert r.end_wall == (None if interactive else r.observation_due_wall)
    at_decision = r.termination_fields(closing=False)
    assert at_decision["termination_observation"]["final"] is None
    if interactive:
        assert r.checkpoint == at_decision
    clock.wall, clock.ros = 10.04, 7.04
    sample(r, 0., 8)
    clock.wall, clock.ros = 11.2, 8.2
    final = r.termination_fields(closing=False)
    observations = final["termination_observation"]
    assert final["final_cmd_vel"] == final["last_odom_twist"] == {1: [0., 0., 0.]}
    assert observations["decision"]["cmd_vel"][1]["velocity"][0] == .0001343523815758571
    assert observations["decision"]["cmd_vel"][1]["received_wall_s"] == 9.
    assert observations["final"]["fresh_cmd_vel_vehicles"] == observations["final"]["fresh_odom_vehicles"] == [1]
    assert observations["final"]["odom"][1]["source_ros_s"] == 8.
    assert observations["final"]["cmd_vel"][1]["received_wall_s"] == 10.04
    assert observations["final"]["window_complete"]
    # The interactive UI can continue to receive data after the experiment has finished.
    clock.wall = 14.
    sample(r, .04, 12)
    assert r.termination_fields(closing=True) == final


def test_interrupted_or_missing_feedback_keeps_last_real_value(monkeypatch):
    r, clock = runner(monkeypatch)
    sample(r, .12, 7)
    clock.wall = 10.
    r.finish("stopped", "fault")
    clock.wall = 10.4
    final = r.termination_fields(closing=True)
    assert final["final_cmd_vel"] == final["last_odom_twist"] == {1: [.12, 0., 0.]}
    observed = final["termination_observation"]["final"]
    assert not observed["window_complete"]
    assert observed["fresh_cmd_vel_vehicles"] == observed["fresh_odom_vehicles"] == []
