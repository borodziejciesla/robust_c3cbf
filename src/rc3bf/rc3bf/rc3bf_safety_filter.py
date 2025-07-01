import numpy as np
from scipy.optimize import minimize

from safety_filter.safety_filter_base import SafetyFilterBase


class RobustSafetyFilter(SafetyFilterBase):
    """
    Implements Control Barrier Function (CBF) of type Collision Cone
    for a unidirectional robot and a moving obstacle.
    Robot model:
    [x_r'; y_r'; theta_r'] = [v_r*cos(theta_r); v_r*sin(theta_r); u]
    where v_r is constant, and u is the control signal.
    Barrier function:
    h(x) = <p,v> + ||v|| * sqrt(||p||^2 - r^2)
    where p is the relative position, and v is the relative velocity.
    r is the obstacle radius.
    where v_r is constant, and u is the control signal.
    Barrier function:
    h(x) = <p,v> + ||v|| * sqrt(||p||^2 - r^2)
    where p is the relative position, and v is the relative velocity.
    r is the obstacle radius.
    """

    def __init__(
        self,
        robot_linear_speed_vr: float,
        obstacle_radius_r: float,
        delta: float = 0.01,
        epsilon: float = 1e-6,
    ):
        """
        Initializes the RobustSafetyFilter with speed and obstacle radius.

        Args:
            robot_linear_speed_vr (float): robot linear speed (v_r).
            obstacle_radius_r (float): Radius of the obstacle (r).
            delta (float): acceptable failure margin.
            epsilon (float): Small value added to denominators to prevent
                             division by zero in case of zero norms.

        Raises:
            ValueError: If obstacle_radius_r or epsilon is less than or equal to zero.
        """
        super().__init__(
            robot_linear_speed_vr, obstacle_radius_r, epsilon
        )
        self.delta = delta

    def run_filter(
        self,
        robot_pose,
        obstacle_pos,
        obstacle_vel,
        u_nominal: float,
    ) -> float:
        """
        Runs safety filter

        Args:
            robot_state (np.ndarray): robot pose [x_r, y_r, theta_r].
            obstacle_pos (np.ndarray): obstacle position [x_o, y_o].
            obstacle_vel (np.ndarray): obstacle velocity [v_ox, v_oy].

        Returns:
            float: barrier function value h(x).
        """
        # Update robot state and obstacle state
        self.robot_pose = robot_pose
        self.obstacle_position = obstacle_pos
        self.obstacle_velocity = obstacle_vel

        # Define constrains
        alpha = 1.0  # Safety margin

        def constraint(u):
            self._calculate_relative_vectors(u)
            return self.h_prim(u) + alpha * self.h() - 1.0

        cons = [
            {
                "type": "ineq",
                "fun": lambda u: constraint(
                    u
                ),  # h'(x) + alpha * h(x) >= 0
            }
        ]

        # Initial point
        self.u_nom = u_nominal

        # Minimize the objective function
        u_safe = minimize(
            self.objective_function, self.u_nom, constraints=cons
        )

        self._calculate_relative_vectors(u_safe.x)

        return u_safe.x

    def objective_function(self, u: float) -> float:
        return (self.u_nom - u) ** 2

    def _calculate_relative_vectors(self, u: float):
        """
        Calculates relative position, velocity, and acceleration vectors

        Args:
            u (float): nominal control.
        """
        x_r, y_r, theta_r = self.robot_pose
        x_o, y_o = self.obstacle_position
        vx_o, vy_o = self.obstacle_velocity

        # Relative position
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
        """
        Calculate barrier function value h(x)

        Returns:
            float: h(x)
        """
        return float(
            np.dot(self.p.T, self.v)
            + np.linalg.norm(self.v)
            * np.sqrt(np.linalg.norm(self.p) ** 2 - self.r**2)
        )

    def h_prim(self, u: float) -> float:
        """
        Calculate h derivative value

        Args:
            u (float): control input.

        Returns:
            float: h(x) derivative value
        """

        return float(
            np.dot(self.v.T, self.v)
            + np.dot(self.p.T, self.a)
            + np.dot(self.v.T, self.a)
            * np.sqrt(np.linalg.norm(self.p) ** 2 - self.r**2)
            / (np.linalg.norm(self.p) + self.epsilon)
            + np.dot(self.p.T, self.v)
            * np.linalg.norm(self.v)
            / np.sqrt(
                np.linalg.norm(self.p) ** 2 - self.r**2 + self.epsilon
            )
        )
