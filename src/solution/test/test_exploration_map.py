import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from exploration_map_node import suppress_pallet_shadows
from maze_grid_node import MazeGrid


class PalletShadowTest(unittest.TestCase):
    def setUp(self):
        self.resolution = 0.05
        self.origin = np.array([-3.0, -3.0])
        yy, xx = np.indices((120, 120))
        self.x, self.y = (xx+0.5)*0.05-3, (yy+0.5)*0.05-3
        self.cells = np.zeros((120, 120), dtype=np.int16)
        self.pallet = (abs(self.x-0.1) < 0.2) & (abs(self.y) < 0.2)
        self.shadow = (self.x > 0.35) & (self.x < 0.8) & (abs(self.y) < 0.2)
        self.exit = (self.x > 0.9) & (self.x < 1.4) & (abs(self.y) < 0.3)
        self.cells[self.pallet] = 100
        self.cells[self.shadow | self.exit] = -1

    def filter(self, cells):
        return suppress_pallet_shadows(cells, self.resolution, self.origin,
                                       np.eye(2), np.zeros(2), np.eye(2))

    def test_only_unknown_interior_shadow_is_suppressed(self):
        original = self.cells.copy()
        filtered, pallets = self.filter(self.cells)
        self.assertEqual(pallets, {(0, 0)})
        self.assertTrue(np.any(filtered[self.shadow] == 100))
        np.testing.assert_array_equal(filtered[self.exit], original[self.exit])
        np.testing.assert_array_equal(filtered[original >= 0], original[original >= 0])
        np.testing.assert_array_equal(self.cells, original)

    def test_no_pallet_keeps_unknown_regions(self):
        self.cells[self.pallet] = 0
        filtered, pallets = self.filter(self.cells)
        self.assertFalse(pallets)
        np.testing.assert_array_equal(filtered, self.cells)

    def test_compact_wall_fragment_at_cell_boundary_is_rejected(self):
        self.cells[self.pallet] = 0
        self.cells[(abs(self.x-1) < 0.2) & (abs(self.y) < 0.2)] = 100
        filtered, pallets = self.filter(self.cells)
        self.assertFalse(pallets)
        np.testing.assert_array_equal(filtered, self.cells)

    def test_new_map_does_not_keep_old_pallet_assumptions(self):
        self.filter(self.cells)
        self.cells[self.pallet] = 0
        filtered, _ = self.filter(self.cells)
        np.testing.assert_array_equal(filtered[self.shadow], self.cells[self.shadow])

    def test_rotated_translated_coordinate_frames(self):
        expected, _ = self.filter(self.cells)
        rotation = MazeGrid.rotation(0.23)
        translation = np.array([1.2, -0.7])
        filtered, pallets = suppress_pallet_shadows(
            self.cells, self.resolution, self.origin @ rotation.T+translation,
            rotation, translation, rotation)
        self.assertEqual(pallets, {(0, 0)})
        np.testing.assert_array_equal(filtered, expected)


if __name__ == '__main__':
    unittest.main()
