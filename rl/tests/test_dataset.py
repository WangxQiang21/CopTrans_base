import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from coptrans_rl.dataset import Corpus, RANGES, generate, make_environment


class DatasetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="coptrans-data-test-", dir="/tmp")
        cls.directory = Path(cls.temporary.name)/"dataset"
        cls.manifest = generate(cls.directory, {"train": 4, "val": 2, "test": 2, "ood": 1})

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_layout_splits_and_certificates(self):
        corpus = Corpus(self.directory)
        self.assertEqual(len(corpus.rows), 9)
        self.assertEqual(len({r["layout_id"] for r in corpus.rows}), 9)
        for row in corpus.rows:
            lo, hi = RANGES[row["split"]]
            self.assertTrue(lo <= row["seed"] < hi)
            self.assertEqual(row["certificate"]["status"], "FEASIBLE")
            self.assertTrue(all(0 <= a < 80 for a in row["action_ids"]))
            p = corpus.problem(row)
            self.assertTrue(p.grid.pose_clear(p.start) and p.grid.pose_clear(p.goal))
        self.assertEqual({r["family"] for r in corpus.split("ood")}, {"islands"})
        self.assertNotIn("islands", {r["family"] for r in corpus.split("train")})

    def test_teachers_replay_in_actual_environment(self):
        corpus = Corpus(self.directory)
        env = make_environment(self.directory, "train")
        for row in corpus.rows:
            env.reset(options={"problem": corpus.problem(row)})
            for j, action in enumerate(row["action_ids"]):
                self.assertTrue(env.action_masks()[action])
                _, _, done, truncated, info = env.step(action)
                self.assertFalse(truncated)
                self.assertEqual(done, j == len(row["action_ids"])-1)
            self.assertTrue(info["is_success"], info)
            self.assertAlmostEqual(info["execution_seconds"],
                                   row["certificate"]["timing"]["duration_s"], places=6)

    def test_tampered_tasks_are_rejected(self):
        path = self.directory/"tasks.jsonl"
        before = path.read_bytes()
        try:
            path.write_bytes(before+b"\n")
            with self.assertRaisesRegex(ValueError, "dataset_tasks_hash_mismatch"):
                Corpus(self.directory)
        finally:
            path.write_bytes(before)

    def test_tampered_manifest_is_rejected(self):
        path = self.directory/"manifest.json"
        before = path.read_bytes()
        try:
            meta = json.loads(before)
            meta["counts"]["train"] += 1
            path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, "dataset_manifest_hash_mismatch"):
                Corpus(self.directory)
        finally:
            path.write_bytes(before)

    def test_tampered_layout_is_rejected(self):
        corpus = Corpus(self.directory)
        identity = corpus.rows[0]["layout_id"]
        path = self.directory/corpus.manifest["layouts"][identity]["file"]
        before = path.read_bytes()
        try:
            path.write_bytes(before+b"corrupt")
            with self.assertRaisesRegex(ValueError, "dataset_layout_file_hash_mismatch"):
                corpus.grid(identity)
        finally:
            path.write_bytes(before)


if __name__ == "__main__":
    unittest.main()
