import unittest
import numpy as np
from behavior_metrics import plan_metrics

TIMING = dict(frequency=25., stable_time=.08, stop_speed=.012)
BASE = dict(limits=dict(wheelbase=.494, track=.364), vehicles=[[0,0,.738,.5]])


def phase(mode, x, y, **kw):
    return dict(dict(mode=mode, kind="motion", duration=2., gear=[1,1,1],
                     heading0=[0.,0.,0.], heading1=[0.,0.,0.], pieces=[[list(x),list(y),[0.]*6]]), **kw)


class BehaviorMetrics(unittest.TestCase):
    def test_normalized_same_gear_stop_inside_but_gear_change_excluded(self):
        x = np.linspace(0.,1.,6)
        first = phase("A", x, 0*x)
        wait = dict(kind="prepare", duration=.12, mode="A")
        second = phase("A", 1+x, 0*x)
        def measure(): return plan_metrics(BASE, [first,wait,second], TIMING)
        m = measure()
        self.assertEqual(("AA","A",1), (m["raw_modes"],m["normalized_modes"],m["internal_stops"]))
        self.assertAlmostEqual(1.,m["longest_continuous_A_m"],places=9)
        second["gear"][0] = -1
        m = measure()
        self.assertEqual(("AA",0),(m["normalized_modes"],m["internal_stops"]))

    def test_parabolic_peak_and_auxiliary_roles(self):
        i = np.arange(6)
        a = phase("A", i/5, .5*i*(i-1)/20)
        m = plan_metrics(BASE,[a],TIMING)
        # At s=0 the analytic curvature of (s,s^2/2) is 1/m and is maximal.
        angles = [abs(np.arctan2(x,1-y)) for x in (.247,-.247) for y in (.182,-.182)]
        np.testing.assert_allclose(m["plan_peak_abs_wheel_angle_rad"][0], angles)
        multi = dict(BASE,vehicles=[[0,0,.738,.5]]*3)
        s = phase("S", i*0, i*0, heading1=[.2,-.3,.4])
        o = phase("O", i*0, i/5)
        m = plan_metrics(multi,[a,s,o],TIMING)
        self.assertAlmostEqual(.9,m["S_angle_rad"])
        self.assertEqual([("S","interior"),("O","terminal")], [(b["mode"],b["role"]) for b in m["auxiliary_roles"]])
        self.assertEqual(2,m["aux_blocks"])
        np.testing.assert_allclose(m["plan_peak_abs_wheel_angle_rad"], np.full((3,4),np.pi/2))


if __name__=="__main__":
    unittest.main()
