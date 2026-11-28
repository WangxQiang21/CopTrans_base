import math

from ranger_mini_gz.body_sim import ACKERMANN, OBLIQUE, SPIN, ZERO, PlantSettings, Response


def test_ackermann_reversal_reaches_zero_and_restarts():
    model = Response(PlantSettings(), managed=True)
    model.velocity = (-.018, 0., .008)
    previous = model.velocity
    for index in range(25):
        result = model.update(index*.02, .02, (.12, 0., .10), None)
        assert previous[0]*result[0] >= 0.
        assert abs(result[0]-previous[0]) <= model.settings.max_accel*.02+1e-10
        assert abs(result[2]-previous[2]) <= model.settings.max_yaw_accel*.02+1e-10
        assert abs(result[0])+1e-10 >= model.settings.switch_radius*abs(result[2])
        previous = result
    assert previous[0] > .08
    assert model.mode == ACKERMANN


def test_managed_switch_waits_at_rest_and_rejects_other_commands():
    model = Response(PlantSettings(), managed=True)
    for index in range(15):
        model.update(index*.02, .02, (.10, 0., 0.), None)
    assert model.velocity[0] > 0.
    model.update(.3, .02, ZERO, SPIN)
    assert model.pending_mode is None
    assert model.mode == ACKERMANN
    for index in range(16, 50):
        model.update(index*.02, .02, ZERO, SPIN)
    assert model.mode == SPIN and model.velocity == ZERO
    assert model.update(1., .02, (.10, 0., 0.), None) == ZERO
    assert model.update(1.02, .02, (0., 0., .1), None)[2] > 0.


def test_manual_adapter_selects_parallel_and_keeps_mode_at_zero():
    model = Response(PlantSettings(), managed=False)
    for index in range(27):
        result = model.update(index*.02, .02, (.10, -.12, .4), None)
    assert model.mode == OBLIQUE
    assert math.hypot(*result[:2]) > 0.1 and result[2] == 0.
    for index in range(27, 50):
        model.update(index*.02, .02, ZERO, None)
    assert model.mode == OBLIQUE and model.velocity == ZERO


def test_large_oblique_turn_brakes_before_crossing_the_steering_limit():
    settings = PlantSettings(max_speed=1.5, high_steer_speed=.75)
    model = Response(settings, managed=True)
    model.mode = OBLIQUE
    dt = .02
    for step in range(600):
        command = (.25, 1.35, 0.) if 200 <= step < 400 else (1.49, .10, 0.)
        old_x, old_y, _ = model.velocity
        x, y, _ = model.update(step * dt, dt, command, None)
        speed = math.hypot(x, y)
        steering = math.atan2(abs(y), abs(x))
        assert speed <= settings.max_speed + 1e-9
        assert steering <= settings.high_steer_angle + 1e-9 or speed <= settings.high_steer_speed + 1e-9
        assert math.hypot(x - old_x, y - old_y) <= settings.max_accel * dt + 1e-9
        if step in (199, 599):
            assert speed > 1.4


def test_blade_turn_keeps_pin_spacing_and_independent_chassis_headings():
    from gz.math7 import Pose3d

    from ranger_mini_gz.blade_system import new_poses, project_mounts

    offsets = ((-5., 0.), (5., 0.))
    projected = project_mounts(offsets, 0., ((.4, -.2), (.8, .2)))
    assert all(math.isclose(value, expected, abs_tol=1e-12)
               for value, expected in zip(projected, (.6, 0., .04, .2), strict=True))
    cargo = Pose3d(2., 3., 0., 0., 0., 0.)
    headings = (-.3, .3)
    cars = [Pose3d(2.+x, 3., .322, 0., 0., yaw) for (x, _), yaw in zip(offsets, headings)]
    yaw_rates = (.025, .04)
    commands = [(.6, 0., yaw_rate) for yaw_rate in yaw_rates]
    (blade, root, tip), residual = new_poses([cargo, *cars], offsets, commands, .1)
    root_x, root_y = root.pos().x(), root.pos().y()
    tip_x, tip_y = tip.pos().x(), tip.pos().y()
    assert residual < 1e-12
    assert math.isclose(math.hypot(tip_x-root_x, tip_y-root_y), 10., abs_tol=1e-12)
    assert math.isclose((root_x+tip_x)/2, blade.pos().x(), abs_tol=1e-12)
    assert math.isclose((root_y+tip_y)/2, blade.pos().y(), abs_tol=1e-12)
    assert math.isclose(root.rot().yaw(), headings[0]+.1*yaw_rates[0], abs_tol=1e-12)
    assert math.isclose(tip.rot().yaw(), headings[1]+.1*yaw_rates[1], abs_tol=1e-12)


def test_blade_mode_waits_for_both_chassis_to_brake_and_confirms_together():
    from ranger_mini_gz.blade_system import update_pair

    responses = [Response(PlantSettings(), managed=True) for _ in range(2)]
    responses[1].velocity = (.2, 0., .06)
    commands = ((.12, -.06, 0.),) * 2
    confirmation = None
    for step in range(50):
        twists = update_pair(responses, step*.02, .02, commands, OBLIQUE)
        assert responses[0].mode == responses[1].mode
        if responses[0].mode == ACKERMANN:
            assert twists[0] == ZERO
            assert twists[1][1] == 0.
        else:
            confirmation = step
            assert twists == [ZERO, ZERO]
            break
    assert confirmation is not None and confirmation > 5
    twists = update_pair(responses, (confirmation+1)*.02, .02, commands, OBLIQUE)
    assert twists[0] == twists[1] and twists[0][0] > 0. and twists[0][1] < 0.
    assert twists[0][2] == 0.


def test_blade_oblique_holds_yaws_under_mismatched_mount_velocities():
    from gz.math7 import Pose3d

    from ranger_mini_gz.blade_system import new_poses, turn

    offsets = ((-5., 0.), (5., 0.))
    headings = (-.2, .3)
    cars = [Pose3d(x, y, .322, 0., 0., yaw) for (x, y), yaw in zip(offsets, headings)]
    world_velocities = ((.12, .25), (.16, .21))
    twists = [(*turn(*v, -yaw), 0.) for v, yaw in zip(world_velocities, headings)]
    (blade, root, tip), residual = new_poses([Pose3d(), *cars], offsets, twists, .1, OBLIQUE)
    assert math.isclose(residual, math.dist(*world_velocities)/2., abs_tol=1e-12)
    assert math.isclose(blade.pos().x(), .014, abs_tol=1e-12)
    assert math.isclose(blade.pos().y(), .023, abs_tol=1e-12)
    assert math.isclose(blade.rot().yaw(), 0., abs_tol=1e-12)
    assert math.isclose(root.rot().yaw(), headings[0], abs_tol=1e-12)
    assert math.isclose(tip.rot().yaw(), headings[1], abs_tol=1e-12)
    assert math.isclose(root.pos().y(), tip.pos().y(), abs_tol=1e-12)
    assert math.isclose(tip.pos().x() - root.pos().x(), 10., abs_tol=1e-12)


def test_blade_spin_rotates_each_chassis_about_its_pin():
    from gz.math7 import Pose3d

    from ranger_mini_gz.blade_system import new_poses

    offsets = ((-5., 0.), (5., 0.))
    original = [Pose3d(8., 4., 0., 0., 0., 0.),
                Pose3d(3., 4., .322, 0., 0., -.2),
                Pose3d(13., 4., .322, 0., 0., .3)]
    rates = ((0., 0., .1), (0., 0., -.2))
    (cargo, first, second), error = new_poses(original, offsets, rates, .2, SPIN)
    assert error == 0.
    assert cargo.pos() == original[0].pos() and cargo.rot() == original[0].rot()
    assert first.pos() == original[1].pos() and second.pos() == original[2].pos()
    assert math.isclose(first.rot().yaw(), -.18, abs_tol=1e-12)
    assert math.isclose(second.rot().yaw(), .26, abs_tol=1e-12)
