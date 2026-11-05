import unittest

import numpy as np

from screw_policy.config import ScrewTaskConfig
from screw_policy.tcp_envelope import validate_chunk


class TestTcpEnvelope(unittest.TestCase):
    def test_valid_row(self):
        cfg = ScrewTaskConfig()
        row = [400, 0, -300, -180, 0, 0, 0]
        out = validate_chunk(row, cfg)
        self.assertEqual(out.shape, (1, 7))

    def test_rejects_below_floor(self):
        cfg = ScrewTaskConfig()
        with self.assertRaises(ValueError):
            validate_chunk([400, 0, -335, -180, 0, 0, 0], cfg)

    def test_chunk_max_100(self):
        cfg = ScrewTaskConfig()
        rows = np.tile([400, 0, -300, -180, 0, 0, 0], (101, 1))
        with self.assertRaises(ValueError):
            validate_chunk(rows, cfg)


if __name__ == "__main__":
    unittest.main()
