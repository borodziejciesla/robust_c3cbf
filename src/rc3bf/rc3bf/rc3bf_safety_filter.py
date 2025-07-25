import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm


class RobustSafetyFilter:
    def __init__(
        self,
        obstacle_radius_r: float,
        epsilon: float = 1e-6,
    ):
        """Initialize the SafetyFilter with robot speed and obstacle radius.

        Args:
            obstacle_radius_r (float): Radius of the obstacle (r).
            epsilon (float): Small value added to denominators to prevent
                             division by zero in case of zero norms.

        Raises:
            ValueError: If obstacle_radius_r or epsilon is less than or equal to zero.
        """
        if obstacle_radius_r <= 0.0:
            raise ValueError(
                "Obstacle radius must be greater than zero for stability."
            )
        if epsilon <= 0.0:
            raise ValueError(
                "Epsilon must be greater than zero for numerical stability."
            )

        self.r = obstacle_radius_r
        self.epsilon = (
            epsilon  # Small value to prevent division by zero
        )

        self.u_nom = (
            0.0  # Nominal control input, not used in this filter
        )
        self.robot_pose = np.array(
            [
                0.0,
                0.0,
                0.0,
            ]
        )  # Robot pose [x_r, y_r, theta_r]
        self.obstacle_position = np.array(
            [
                0.0,
                0.0,
            ]
        )  # Obstacle position [x_o, y_o]
        self.obstacle_velocity = np.array(
            [
                0.0,
                0.0,
            ]
        )  # Obstacle velocity [v_ox, v_oy]

        self.p = np.array([0.0, 0.0])  # Relative position vector
        self.v = np.array([0.0, 0.0])  # Relative velocity vector
        self.a = np.array([0.0, 0.0])  # Relative acceleration vector

    def run_filter(
        self,
        robot_pose,
        obstacle_pos,
        obstacle_vel,
        u_nominal: float,
        v_r: float,
    ) -> float:
        self.robot_pose = robot_pose
        self.obstacle_position = obstacle_pos
        self.obstacle_velocity = obstacle_vel
        self.v_r = v_r

        alpha = 0.1  # Safety margin

        def constraint(u):
            self._calculate_relative_vectors(u)
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

    def _calculate_relative_vectors(self, u):
        x_r, y_r, theta_r = self.robot_pose
        x_o, y_o = self.obstacle_position
        vx_o, vy_o = self.obstacle_velocity

        self.p = np.array([x_o - x_r, y_o - y_r])
        self.v = np.array(
            [
                vx_o - self.v_r * np.cos(theta_r),
                vy_o - self.v_r * np.sin(theta_r),
            ]
        )
        self.a = np.array(
            [
                self.v_r * u * np.sin(theta_r),
                -self.v_r * u * np.cos(theta_r),
            ]
        )

    def h(self) -> float:
        return float(
            float(self.p.T @ self.v)
            + np.linalg.norm(self.v)
            * np.sqrt(
                max(np.linalg.norm(self.p) ** 2 - self.r**2, 1e-6)
            )
        )

    def h_prim(self, u: float) -> float:
        return float(
            np.sum(self.v**2)
            + float(self.p.T @ self.a)
            + float(self.v.T @ self.a)
            * np.sqrt(
                max(np.linalg.norm(self.p) ** 2 - self.r**2, 1e-6)
            )
            / (np.linalg.norm(self.p) + self.epsilon)
            + float(self.p.T @ self.v)
            * np.linalg.norm(self.v)
            / np.sqrt(
                max(np.linalg.norm(self.p) ** 2 - self.r**2, 1e-6)
            )
        )

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

        sigma_p = np.array([[0.01, 0.0], [0.0, 0.01]])
        sigma_v = np.array([[0.01, 0.0], [0.0, 0.01]])

        sp = nabla_pr_h + nabla_pr_hp
        sv = nabla_vr_h + nabla_vr_hp
        # Calculate the variance of the safety function h
        # using the law of total variance
        s = sp.T @ sigma_p @ sp + sv.T @ sigma_v @ sv
        s_scalar = float(s)
        return s_scalar
