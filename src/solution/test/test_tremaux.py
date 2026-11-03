import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from tremaux_node import Tremaux


class TremauxTest(unittest.TestCase):
    def walk(self, edges):
        adjacency = {}
        for a, b in edges:
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)
        graph = Tremaux()
        graph.arrive((0, 0))
        for _ in range(2*len(edges)+1):
            target = graph.choose(adjacency[graph.current])
            if target is None:
                break
            graph.arrive(target)
        else:
            self.fail('Did not finish in two traversals per edge')
        self.assertEqual(graph.current, (0, 0))
        self.assertEqual(graph.seen, set(adjacency))
        self.assertEqual(set(graph.marks.values()), {2})
        return graph

    def test_dead_ends_and_branches(self):
        self.walk([((0, 0), (1, 0)), ((1, 0), (2, 0)), ((1, 0), (1, 1))])

    def test_cycle_with_branch_does_not_loop(self):
        self.walk([((0, 0), (1, 0)), ((1, 0), (1, 1)),
                   ((1, 1), (0, 1)), ((0, 1), (0, 0)), ((1, 1), (2, 1))])

    def test_connected_grid_with_many_cycles(self):
        edges = []
        for x in range(4):
            for y in range(4):
                if x < 3:
                    edges.append(((x, y), (x+1, y)))
                if y < 3:
                    edges.append(((x, y), (x, y+1)))
        self.walk(edges)

    def test_stationary_updates_do_not_mark_passage(self):
        graph = Tremaux()
        graph.arrive((0, 0))
        for _ in range(10):
            graph.arrive((0, 0))
        self.assertEqual(graph.marks, {})
        self.assertEqual(graph.path, [(0, 0)])


if __name__ == '__main__':
    unittest.main()
