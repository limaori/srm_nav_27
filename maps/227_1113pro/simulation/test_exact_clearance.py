"""Geometry regressions for the independent 0.30 m acceptance measurement."""
import unittest
import numpy as np
from exact_clearance import path_clearance, segment_boxes


class ExactClearanceTest(unittest.TestCase):
    def test_segment_crossing_with_both_endpoints_outside(self):
        self.assertEqual(segment_boxes([-2,0],[2,0],np.array([[0.,0.]]),1)[0],0)

    def test_parallel_segment_to_square_edge(self):
        self.assertAlmostEqual(segment_boxes([-2,2],[2,2],np.array([[0.,0.]]),1)[0],1)

    def test_diagonal_corner_distance_is_euclidean(self):
        self.assertAlmostEqual(segment_boxes([2,2],[3,3],np.array([[0.,0.]]),1)[0],2**.5)

    def test_zero_length_segment(self):
        self.assertAlmostEqual(segment_boxes([2,2],[2,2],np.array([[0.,0.]]),1)[0],2**.5)

    def test_diagonal_clearance_below_gate_is_rejected(self):
        # L1 would report .40 m and incorrectly pass; Euclidean is .2828 m.
        self.assertLess(segment_boxes([1.2,1.2],[2,2],np.array([[0.,0.]]),1)[0],.30)

    def test_cell_culling_keeps_nearest_obstacle_at_segment_interior(self):
        costs=np.zeros((20,20),dtype=int);costs[10,10]=100;costs[0,0]=100
        result=path_clearance([[3.,12.],[18.,12.]],costs,1.,[0.,0.])
        self.assertAlmostEqual(result['min_m'],1.)

    def test_segment_splitting_does_not_change_clearance(self):
        costs=np.zeros((20,20),dtype=int);costs[10,10]=100
        a=path_clearance([[3.,12.],[18.,12.]],costs,1.,[0.,0.])
        b=path_clearance([[3.,12.],[9.,12.],[18.,12.]],costs,1.,[0.,0.])
        self.assertAlmostEqual(a['min_m'],b['min_m'])

    def test_outside_map_cannot_pass_gate(self):
        costs=np.zeros((20,20),dtype=int);costs[10,10]=100
        self.assertEqual(path_clearance([[-1.,12.],[18.,12.]],costs,1.,[0.,0.])['min_m'],0)


if __name__=='__main__':
    unittest.main()
