import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cell_explorer_node import CellExplorer, edge_key, extend_corridor, path_metrics, reachable_cells


class CellExplorerTest(unittest.TestCase):
    def test_reachable_uses_open_passages_and_not_unknown(self):
        edges = {edge_key((0, 0), (1, 0)): 'wall',
                 edge_key((0, 0), (0, 1)): 'open',
                 edge_key((0, 1), (1, 1)): 'open',
                 edge_key((1, 1), (1, 0)): 'open'}
        parents = reachable_cells((0, 0), edges)
        self.assertEqual(parents[(1, 0)], (1, 1))
        self.assertNotIn((-1, 0), parents)

    def test_corridor_extends_but_stops_at_junction(self):
        edges = {edge_key((x, 0), (x+1, 0)): 'open' for x in range(5)}
        edges[edge_key((3, 0), (3, 1))] = 'open'
        self.assertEqual(extend_corridor((1, 0), (0, 0), edges, 5), (3, 0))
        self.assertEqual(extend_corridor((1, 0), (0, 0), edges, 2), (2, 0))

    def test_route_distance_and_initial_turn(self):
        path = NavPath()
        for x, y in [(0, 0), (0, 2), (2, 2), (2, 0)]:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            path.poses.append(p)
        length, turn = path_metrics(path, (0, 0), 0)
        self.assertAlmostEqual(length, 6.0)
        self.assertAlmostEqual(turn, math.pi/2)

    def test_goal_avoids_pallet_and_unknown_space(self):
        cells = np.zeros((100, 100), dtype=np.uint8)
        cells[:10] = cells[-10:] = 1
        cells[:, :10] = cells[:, -10:] = 1
        cells[40:60, 40:60] = 1
        free = (cells == 0).astype(np.uint8)
        fixture = SimpleNamespace(clearances=cv2.distanceTransform(free, cv2.DIST_L2, 5)*0.02,
            resolution=0.02, map_rotation=np.eye(2), map_origin=np.array([-1., -1.]),
            grid_origin=np.zeros(2), grid_rotation=np.eye(2), param=lambda _: 0.25)
        point = CellExplorer.free_point(fixture, (0, 0))
        self.assertIsNotNone(point)
        ix, iy = np.floor((np.array(point)+1)/0.02).astype(int)
        self.assertGreaterEqual(fixture.clearances[iy, ix], 0.25)
        self.assertGreater(math.hypot(*point), 0.35)

    def test_cell_without_safe_space_has_no_goal(self):
        fixture = SimpleNamespace(clearances=np.zeros((10, 10)), resolution=0.05,
            map_rotation=np.eye(2), map_origin=np.zeros(2), grid_origin=np.zeros(2),
            grid_rotation=np.eye(2), param=lambda _: 0.25)
        self.assertIsNone(CellExplorer.free_point(fixture, (0, 0)))


if __name__ == '__main__':
    unittest.main()
