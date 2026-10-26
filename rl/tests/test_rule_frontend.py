import dataclasses
import math
import unittest

import numpy as np

from coptrans_rl.kernel_adapter import Action,Grid,Pose,load_contract,replay
from coptrans_rl.local_connections import LocalConnections
from coptrans_rl.rule_candidates import RuleCandidates
from coptrans_rl.rule_frontend import RuleFrontend,FrontendConfig,solve


class RuleFrontendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_contract()
        cls.grid = Grid(np.zeros((160,160),bool),.05,0.,0.,
                        cls.contract.length,cls.contract.width,cls.contract.margin)

    def problem(self, actions):
        start = Pose(3.,3.,0.)
        return self.contract.problem(self.grid,start,replay(start,actions)[-1])

    def test_native_local_connections_replay_exactly(self):
        connector = LocalConnections()
        start = Pose(3.,3.,.2)
        for actions in ([Action(0,1.,0.)],[Action(0,1.,1/.81)]):
            goal = replay(start,actions)[-1]
            options = list(connector.ackermann(start,goal,.81))
            self.assertTrue(options)
            for option in options:
                end = replay(start,option)[-1]
                self.assertLess(math.hypot(end.x-goal.x,end.y-goal.y),1e-6)
                self.assertTrue(all(abs(a.control)<=1/.81+1e-8 for a in option))

    def test_candidates_are_swept_and_respect_direction(self):
        problem = dataclasses.replace(self.problem([Action(1,1.,math.pi/2)]),allow_reverse=False)
        candidates = RuleCandidates(problem).generate(problem.start)
        self.assertTrue(candidates)
        for candidate in candidates:
            self.assertFalse(any(a.mode!=2 and a.amount<0 for a in candidate.actions))
            p = problem.start
            for action in candidate.actions:
                self.assertTrue(problem.grid.action_clear(p,action,.0025))
                p = replay(p,[action])[-1]

    def test_rule_baseline_certifies_basic_modes(self):
        config = FrontendConfig(budget_s=5.)
        for actions in ([Action(0,1.,0.)],[Action(1,1.,math.pi/2)],
                        [Action(2,math.pi/2,0.)]):
            report = solve(self.problem(actions),config,include_trace=False)
            self.assertTrue(report["is_success"],report["reason"])
            self.assertEqual(report["finalization"]["audit"]["geometry_status"],"PASS")
            self.assertEqual(report["finalization"]["audit"]["dynamic_status"],"PASS")

    def test_ackermann_corner_remains_available_and_selected(self):
        problem = self.problem([Action(0,.81*math.pi/2,1/.81)])
        result = solve(problem,FrontendConfig(budget_s=5.),include_trace=False)
        self.assertTrue(result["is_success"])
        self.assertTrue(all(a[0]==0 for a in result["finalization"]["actions"]))

    def test_frontier_observation_and_invalid_choice(self):
        env = RuleFrontend(FrontendConfig(budget_s=5.))
        obs = env.reset(self.problem([Action(0,1.,0.)]))
        self.assertEqual(obs["candidates"].shape,(64,27))
        self.assertEqual(obs["map"].shape,(2,32,32))
        self.assertTrue(all(np.isfinite(v).all() for v in obs.values()))
        for bad in (-1,True,1.5,64):
            with self.assertRaises(ValueError):
                env.step(bad)

    def test_deadline_and_configuration_are_explicit(self):
        for kwargs in ({"capacity":0},{"max_decisions":True},{"budget_s":float("nan")},
                       {"budget_s":1.,"finalize_reserve_s":2.}):
            with self.assertRaises(ValueError):
                FrontendConfig(**kwargs)

    def test_zero_residual_and_permutation_equivariance(self):
        import torch
        from coptrans_rl.rule_learning import CandidateRanker,tensors
        torch.set_num_threads(1)
        env = RuleFrontend(FrontendConfig(budget_s=5.))
        obs = env.reset(self.problem([Action(0,1.,0.)]))
        policy = CandidateRanker()
        distribution,value = policy(tensors(obs,"cpu"))
        expected = np.argmax(np.where(obs["mask"],obs["rule_logits"],-np.inf))
        self.assertEqual(int(distribution.logits.argmax()),expected)
        self.assertTrue(torch.all(distribution.probs[0,~torch.tensor(obs["mask"])]==0))
        permutation = np.random.default_rng(1).permutation(64)
        swapped = {k:(v[permutation] if k!="map" else v) for k,v in obs.items()}
        other,other_value = policy(tensors(swapped,"cpu"))
        np.testing.assert_allclose(distribution.probs.detach().numpy()[0,permutation],
                                   other.probs.detach().numpy()[0],rtol=1e-5,atol=1e-7)
        torch.testing.assert_close(value,other_value)
        loss = -distribution.log_prob(distribution.probs.argmax(dim=1)).mean()+value.square().mean()
        loss.backward()
        self.assertTrue(all(torch.isfinite(p.grad).all() for p in policy.parameters() if p.grad is not None))


if __name__=="__main__":
    unittest.main()
