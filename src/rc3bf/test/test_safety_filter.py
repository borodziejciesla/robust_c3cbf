import unittest
import numpy as np
from rc3bf.safety_filter import SafetyFilter


class TestSafetyFilter(unittest.TestCase):
    def setUp(self):
        self.r_ = 0.5
        self.epsilon_ = 1e-6
        # Initialize the SafetyFilter with a radius and epsilon
        # These values are chosen to be valid for the tests
        self.filter = SafetyFilter(
            obstacle_radius_r=self.r_, epsilon=self.epsilon_
        )

    def test_initialization(self):
        self.assertEqual(self.filter.r, 0.5)
        self.assertEqual(self.filter.epsilon, 1e-6)

    def test_run_filter_obstacle_outside_colision_cone(self):
        robot_pose = [0.0, 0.0, 0.0]
        obstacle_pos = [1.0, 1.0]
        obstacle_vel = [0.1, 0.1]
        u_nominal = 1.0
        v_r = 2.0

        u = self.filter.run_filter(
            robot_pose, obstacle_pos, obstacle_vel, u_nominal, v_r
        )
        self.assertIsNotNone(u)
        h = self.filter.h()
        self.assertGreater(h, 0.0)
        h_prim = self.filter.h_prim(u)
        self.assertGreater(-h_prim, 0.0)
        self.assertGreater(h + 0.1 * h_prim, 0.0)

    def test_run_filter_obstacle_in_colision_cone(self):
        robot_pose = [0.0, 0.0, 0.0]
        obstacle_pos = [0.1, 0.0]
        obstacle_vel = [0.1, 0.1]
        u_nominal = 1.0
        v_r = 20.0

        u = self.filter.run_filter(
            robot_pose, obstacle_pos, obstacle_vel, u_nominal, v_r
        )
        self.assertIsNone(u)

    def test_h_function_value(self):
        robot_pose = [0.0, 0.0, 0.0]
        obstacle_pos = [1.0, 1.0]
        obstacle_vel = [0.0, 0.0]
        u_nominal = 1.0
        v_r = 2.0

        self.filter.run_filter(
            robot_pose, obstacle_pos, obstacle_vel, u_nominal, v_r
        )
        # Caculate reference h value
        p = np.array(
            [
                obstacle_pos[0] - robot_pose[0],
                obstacle_pos[1] - robot_pose[1],
            ]
        )
        v = np.array(
            [
                -v_r,
                0.0,
            ]
        )
        h = np.dot(p.T, v) + np.linalg.norm(v) * np.sqrt(
            np.linalg.norm(p) ** 2 - self.r_**2
        )
        h_ref = float(h)
        h_cbf = self.filter.h()
        self.assertAlmostEqual(h_ref, h_cbf, delta=self.epsilon_)

    def test_invalid_obstacle_radius(self):
        with self.assertRaises(ValueError):
            SafetyFilter(obstacle_radius_r=-1.0)

    def test_invalid_epsilon(self):
        with self.assertRaises(ValueError):
            SafetyFilter(obstacle_radius_r=0.5, epsilon=-1e-6)
