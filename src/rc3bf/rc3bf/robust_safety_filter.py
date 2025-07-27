import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm

from .safety_filter import SafetyFilter


class RobustSafetyFilter(SafetyFilter):
    def __init__(
        self,
        obstacle_radius_r: float,
        epsilon: float = 1e-6,
    ):
        """Initialize the RobustSafetyFilter with robot speed and obstacle radius.

        Args:
            obstacle_radius_r (float): Radius of the obstacle (r).
            epsilon (float): Small value added to denominators to prevent
                             division by zero in case of zero norms.

        Raises:
            ValueError: If obstacle_radius_r or epsilon is less than or equal to zero.
        """
        super().__init__(obstacle_radius_r, epsilon)

        self.obstacle_position_cov = np.array(
            [[0.0, 0.0], [0.0, 0.0]]
        )  # Obstacle position covariance [xx, xy; xy yy]
        self.robot_position_cov = np.array(
            [[0.0, 0.0], [0.0, 0.0]]
        )  # Robot position covariance [xx, xy; xy yy]
        self.obstacle_velocity_cov = np.array(
            [[0.0, 0.0], [0.0, 0.0]]
        )  # Obstacle velocity covariance [xx, xy; xy yy]

        self.p_cov = np.array(
            [[0.0, 0.0], [0.0, 0.0]]
        )  # position covariance [xx, xy; xy yy]
        self.v_cov = np.array(
            [[0.0, 0.0], [0.0, 0.0]]
        )  # velocity covariance [xx, xy; xy yy]

    def run_filter(
        self,
        robot_pose,
        robot_pose_cov,
        obstacle_pos,
        obstacle_pose_cov,
        obstacle_vel,
        obstacle_vel_cov,
        u_nominal: float,
        v_r: float,
    ) -> float:
        self.robot_pose = robot_pose
        self.robot_position_cov = robot_pose_cov
        self.obstacle_position = obstacle_pos
        self.obstacle_position_cov = obstacle_pose_cov
        self.obstacle_velocity = obstacle_vel
        self.obstacle_velocity_cov = obstacle_vel_cov
        self.v_r = v_r

        alpha = 0.1  # Safety margin

        def constraint(u):
            self._calculate_relative_vectors(u)
            self._calculate_relative_covariances(u)
            sigma = self.sigma_h()
            val = (
                self.h_prim(u)
                + alpha * self.h()
                - norm.ppf(0.95) * sigma
            )
            return np.array([val])

        cons = [
            {
                "type": "ineq",
                "fun": constraint,
            }
        ]

        # Initial point
        self.u_nom = u_nominal

        # Minimize the objective function
        u_safe_result = minimize(
            self.objective_function, self.u_nom, constraints=cons
        )

        if u_safe_result.success:
            self._calculate_relative_vectors(u_safe_result.x)
            return u_safe_result.x
        else:
            return None

    def objective_function(self, u):
        # minimize the squared difference from the nominal control input
        val = (self.u_nom - u[0]) ** 2
        return val

    def _calculate_relative_covariances(self, u):
        """Calculates relative position and velocity covariance matrices.
        Args:
            u (float): nominal control.
        """
        # Calculate the relative position and velocity covariance matrices
        self.p_cov = (
            self.obstacle_position_cov + self.robot_position_cov
        )
        self.v_cov = self.obstacle_velocity_cov

    def sigma_h(self) -> float:
        delta = np.sqrt(
            max(np.linalg.norm(self.p) ** 2 - self.r**2, 1e-6)
        )

        nabla_pr_h = (
            self.v + (np.linalg.norm(self.v) / delta) * self.p
        )
        nabla_vr_h = (
            self.p + (delta / np.linalg.norm(self.v)) * self.v
        )

        # nabla_pr_hp and nabla_vr_hp are the gradients of h_prim with respect to p and v_r
        # They are calculated using the chain rule and the properties of the h_prim function
        nabla_pr_hp = (np.linalg.norm(self.v) / delta) * self.v - (
            np.linalg.norm(self.v) * float(self.p.T @ self.v) * self.p
        ) / delta**3
        nabla_vr_hp = (
            2 * self.v
            + (
                np.linalg.norm(self.v) * self.p
                + self.p.T @ self.v * self.v / np.linalg.norm(self.v)
            )
            / delta
        )

        sp = nabla_pr_h + nabla_pr_hp
        sv = nabla_vr_h + nabla_vr_hp
        # Calculate the variance of the safety function h
        # using the law of total variance
        s = sp.T @ self.p_cov @ sp + sv.T @ self.v_cov @ sv
        s_scalar = float(s)
        return s_scalar
