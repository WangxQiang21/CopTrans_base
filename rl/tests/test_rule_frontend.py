import dataclasses
import math
import unittest
import json
import tempfile
from pathlib import Path

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

    def test_offline_reference_inputs_are_checked(self):
        from PIL import Image
        import yaml
        from coptrans_rl.rule_reference_audit import check_reference
        problem = self.problem([Action(0,1.,0.)])
        probe = {"start":[3.,3.,0.],"goal":[4.,3.,0.]}
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            root = Path(directory); maps = root/"runtime_map"; maps.mkdir()
            Image.fromarray(np.full((160,160),255,dtype=np.uint8)).save(maps/"map.png")
            (maps/"map.yaml").write_text(yaml.safe_dump({"image":"map.png","resolution":.05,"origin":[0.,0.,0.]}))
            check_reference(problem,root,probe)
            wrong = dict(probe,start=[3.1,3.,0.])
            with self.assertRaisesRegex(ValueError,"reference_start_mismatch"):
                check_reference(problem,root,wrong)
            pixels = np.full((160,160),255,dtype=np.uint8); pixels[20,20]=0
            Image.fromarray(pixels).save(maps/"map.png")
            with self.assertRaisesRegex(ValueError,"reference_occupancy_mismatch"):
                check_reference(problem,root,probe)

    def test_reference_oracle_selects_only_available_candidates(self):
        from coptrans_rl.rule_reference_audit import ReferenceOracle
        problem = self.problem([Action(0,1.,0.)])
        trace = [[i*.1,3.+i*.1,3.,0.,0.,0] for i in range(11)]
        env = RuleFrontend(FrontendConfig(budget_s=5.)); env.reset(problem)
        oracle = ReferenceOracle(trace,problem)
        while not env.done:
            index,station,error = oracle.choose(env)
            self.assertTrue(0<=index<len(env.frontier))
            self.assertTrue(0<=station<len(oracle.poses))
            self.assertTrue(math.isfinite(error))
            env.step(index)
        self.assertTrue(env.report(False)["is_success"])

    def test_offline_teacher_masks_certificates_and_task_holdout(self):
        from coptrans_rl.rule_imitation import load_teachers
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            path = Path(directory)
            names = ["map01-example","map06-example"]
            rows = [{"task_id":name,"is_success":True,"finalization":{"status":"FEASIBLE",
                     "audit":{key:"PASS" for key in ("geometry_status","dynamic_status","timing_status")}}}
                    for name in names]
            report = {"method":"reference_oracle_diagnostic","tasks":rows}
            (path/"evaluation.json").write_text(json.dumps(report))
            arrays = {"candidates":np.zeros((2,64,27),np.float32),"rule_logits":np.zeros((2,64),np.float32),
                      "mask":np.ones((2,64),bool),"map":np.zeros((2,2,32,32),np.float32),
                      "labels":np.array([0,1]),"task_ids":np.asarray(names)}
            np.savez_compressed(path/"teacher_observations.npz",**arrays)
            _,_,_,train,holdout = load_teachers(path,["map06"])
            np.testing.assert_array_equal(train,[True,False])
            np.testing.assert_array_equal(holdout,[False,True])
            arrays["mask"][0,0] = False
            np.savez_compressed(path/"teacher_observations.npz",**arrays)
            with self.assertRaisesRegex(ValueError,"masked_teacher_label"):
                load_teachers(path)
            arrays["mask"][0,0] = True
            np.savez_compressed(path/"teacher_observations.npz",**arrays)
            rows[0]["finalization"]["audit"]["geometry_status"] = "FAIL"
            (path/"evaluation.json").write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,"teacher_task_lacks_terminal_certificate"):
                load_teachers(path)


if __name__=="__main__":
    unittest.main()
