import os
import unittest

import numpy as np

# Disable Cosmos for unit tests.
os.environ["REAL_ROBOT_ENABLE_COSMOS"] = "0"

from policy import Policy  # noqa: E402


def _obs():
    h, w = 360, 640
    return {
        "images": {"table": np.zeros((h, w, 3), dtype=np.uint8)},
        "depth": {"table": np.full((400, 640), 1000, dtype=np.uint16)},
        "joints": np.zeros(7, dtype=np.float32),
        "trial": {"status": "running", "trial_id": "test", "instruction": "screws"},
    }


class TestStateMachine(unittest.TestCase):
    def test_act_returns_tcp_row(self):
        p = Policy()
        p.reset()
        out = p.act(_obs())
        arr = np.asarray(out, float)
        self.assertEqual(arr.shape[-1], 7)
        self.assertTrue(np.all(np.isfinite(arr)))

    def test_many_steps_stay_in_envelope(self):
        p = Policy()
        p.reset()
        for _ in range(40):
            out = p.act(_obs())
            row = np.asarray(out, float).reshape(-1, 7)[0]
            self.assertGreaterEqual(row[2], -334.0)


if __name__ == "__main__":
    unittest.main()
