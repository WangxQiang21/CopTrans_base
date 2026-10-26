import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

AVAILABLE = all(importlib.util.find_spec(name) for name in ("torch", "sb3_contrib"))


class FigureSelectionTests(unittest.TestCase):
    def test_includes_late_success_and_failures_without_duplicates(self):
        from coptrans_rl.training_figures import representative_episodes
        rows = [{"task_id": str(i), "info": {"is_success": i >= 8}} for i in range(10)]
        selected = representative_episodes(rows)
        self.assertEqual([r["task_id"] for r in selected], ["8", "9", "0", "1", "2", "3"])
        self.assertEqual(representative_episodes([]), [])
        self.assertEqual(representative_episodes(rows[8:]), rows[8:])


@unittest.skipUnless(AVAILABLE, "training dependencies are optional for foundation tests")
class TrainingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        from coptrans_rl.dataset import generate
        torch.set_num_threads(1)
        cls.temporary = tempfile.TemporaryDirectory(prefix="coptrans-model-test-", dir="/tmp")
        cls.directory = Path(cls.temporary.name)
        cls.data = cls.directory/"dataset"
        generate(cls.data, {"train": 2, "val": 1, "test": 0, "ood": 0})

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_policy_mask_gradient_and_checkpoint_roundtrip(self):
        import torch
        from sb3_contrib import MaskablePPO
        from stable_baselines3.common.vec_env import DummyVecEnv
        from coptrans_rl.dataset import Corpus, make_environment
        from coptrans_rl.train import build_model, run_manifest, save_model
        from coptrans_rl.evaluate import check_model_manifest
        corpus = Corpus(self.data)
        env = DummyVecEnv([lambda: make_environment(self.data)])
        args = argparse.Namespace(device="cpu", seed=7, rollout_steps=16,
                                  batch_size=8, ppo_epochs=1, learning_rate=3e-4, stage="unit")
        try:
            model = build_model(env, args)
            observation = env.reset()
            mask = env.env_method("action_masks")[0]
            tensors, _ = model.policy.obs_to_tensor(observation)
            distribution = model.policy.get_distribution(tensors, action_masks=mask[None, :])
            probabilities = distribution.distribution.probs.detach().cpu().numpy()[0]
            self.assertTrue(np.all(probabilities[~mask] == 0.))
            action = int(np.flatnonzero(mask)[0])
            parameter = next(model.policy.parameters())
            before = parameter.detach().clone()
            loss = -distribution.log_prob(torch.tensor([action])).mean()
            model.policy.optimizer.zero_grad(); loss.backward(); model.policy.optimizer.step()
            self.assertFalse(torch.equal(before, parameter))
            output = self.directory/"model"
            output.mkdir()
            run_manifest(args, corpus, output)
            save_model(model, output, "best_model")
            check_model_manifest(output/"best_model.zip", corpus)
            loaded = MaskablePPO.load(output/"best_model.zip", device="cpu")
            expected = model.predict(observation, deterministic=True, action_masks=mask[None, :])[0]
            actual = loaded.predict(observation, deterministic=True, action_masks=mask[None, :])[0]
            np.testing.assert_array_equal(expected, actual)
            with (output/"best_model.zip").open("ab") as stream:
                stream.write(b"corrupt")
            with self.assertRaisesRegex(ValueError, "model_checkpoint_hash_mismatch"):
                check_model_manifest(output/"best_model.zip", corpus)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
