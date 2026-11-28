import pytest
import shapely as sg

from run import SRC, comparison
from scenes import study_scene


@pytest.mark.parametrize("bend,width,section,unaffected", [
    ("return", 4.2, [(58.74167, 47.5), (67.40192, 52.5)], sg.box(80, 38, 125, 61)),
    ("hairpin", 5.4, [(110,38),(118,38)], sg.box(20, 40, 89, 60)),
])
def test_mountain_experiment_only_trims_the_existing_road(bend, width, section, unaffected):
    base, scene, _, settings = study_scene(SRC, "mountain", width, bend=bend)
    assert base.blade_site.centreline.equals_exact(scene.blade_site.centreline, 0.)
    assert scene.blade_site.road.difference(base.blade_site.road).area < 1e-10
    outside_edit = base.blade_site.road.intersection(unaffected)
    assert outside_edit.symmetric_difference(scene.blade_site.road.intersection(unaffected)).area < 1e-10
    assert scene.blade_site.road.intersection(sg.LineString(section)).length == pytest.approx(width, abs=.003)
    assert scene.experiments["blade_pair"] == base.experiments["blade_pair"]
    assert settings["centerline_min_radius_m"] == 8.


def test_multimode_inherits_reversible_ackermann_solution():
    accepted=dict(certified=True,switches=0,gear_changes_per_car=[4,4],chassis_travel_m=[12.,13.])
    selection=comparison({"signed":accepted,"sideways":{"certified":False}})
    assert selection["pure_ackermann"]==selection["multimode_including_ackermann"]=="signed"


def test_reprofiled_reverse_curves_join_the_original_approaches():
    base, scene, _, _ = study_scene(SRC, "mountain", 4.2, return_radius=6.)
    window = sg.box(48., 40., 77., 61.)
    assert scene.blade_site.road.geom_type == "Polygon"
    assert scene.blade_site.road.symmetric_difference(base.blade_site.road).difference(window).area < 1e-10
    for x, y, inside in ((48., 54., .1), (77., 46., -.1)):
        for d in (0., inside, -inside):
            q = x + d
            crossing = scene.blade_site.road.intersection(sg.LineString([(q, y-4.), (q, y+4.)]))
            assert crossing.length == pytest.approx(5.8, abs=.0001)
