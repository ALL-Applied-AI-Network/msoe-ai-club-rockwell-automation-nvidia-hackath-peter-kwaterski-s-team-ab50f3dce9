import unittest

import numpy as np

from screw_policy.geometry import base_to_uf0_mm


class TestGeometry(unittest.TestCase):
    def test_base_to_uf0_z(self):
        pts = np.array([[0.0, 0.0, -0.0051]])
        uf0 = base_to_uf0_mm(pts)[0]
        self.assertAlmostEqual(uf0[2], -335.1, places=1)


if __name__ == "__main__":
    unittest.main()
