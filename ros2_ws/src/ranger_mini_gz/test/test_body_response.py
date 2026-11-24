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
