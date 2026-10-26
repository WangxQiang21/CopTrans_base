"""Exercise the actual four-worker masked sampling path before PPO."""
import argparse
from functools import partial
import json
import time

import numpy as np

from coptrans_rl.dataset import make_environment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--steps", type=int, default=16)
    args = parser.parse_args()
    from stable_baselines3.common.vec_env import SubprocVecEnv
    begun = time.perf_counter()
    env = SubprocVecEnv([partial(make_environment, args.dataset, "train", 32)
                         for _ in range(args.workers)], start_method="forkserver")
    try:
        env.seed(7)
        observations = env.reset()
        assert all(np.isfinite(v).all() for v in observations.values())
        for _ in range(args.steps):
            masks = np.asarray(env.env_method("action_masks"))
            assert masks.shape == (args.workers, 80) and masks.any(axis=1).all()
            actions = np.asarray([np.flatnonzero(mask)[0] for mask in masks])
            observations, rewards, done, infos = env.step(actions)
            assert np.isfinite(rewards).all()
            assert all(np.isfinite(v).all() for v in observations.values())
        print(json.dumps({"passed": True, "workers": args.workers,
                          "transitions": args.workers*args.steps,
                          "wall_s_including_startup": time.perf_counter()-begun}))
    finally:
        env.close()


if __name__ == "__main__":
    main()
