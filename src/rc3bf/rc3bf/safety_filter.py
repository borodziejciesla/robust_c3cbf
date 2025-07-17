import numpy as np
from scipy.optimize import minimize


class SafetyFilter:
    """Implements Control Barrier Function (CBF) of type Collision Cone.
    For a unidirectional robot and a moving obstacle.
    Robot model:
    [x_r'; y_r'; theta_r'] = [v_r*cos(theta_r); v_r*sin(theta_r); u]
    where v_r is constant, and u is the control signal.
    Barrier function:
    h(x) = <p,v> + ||v|| * sqrt(||p||^2 - r^2)
    where p is the relative position, and v is the relative velocity.
    r is the obstacle radius.
    """

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
        """Run safety filter.

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
        self.v_r = v_r

        # Define constrains
        alpha = 0.1  # Safety margin

        def constraint(u):
            self._calculate_relative_vectors(u)
            return self.h_prim(u) + alpha * self.h()

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
        u_safe_result = minimize(
            self.objective_function, self.u_nom, constraints=cons
        )

        if u_safe_result.success:
            self._calculate_relative_vectors(u_safe_result.x)
            return u_safe_result.x
        else:
            return None

    def objective_function(self, u: float) -> float:
        return (self.u_nom - u) ** 2

    def _calculate_relative_vectors(self, u: float):
        """Calculates relative position, velocity, and acceleration vectors.

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
        """Calculate barrier function value h(x).

        Returns:
            float: h(x)
        """
        return float(
            np.dot(self.p.T, self.v)
            + np.linalg.norm(self.v)
            * np.sqrt(np.linalg.norm(self.p) ** 2 - self.r**2)
        )

    def h_prim(self, u: float) -> float:
        """Calculate h derivative value.

        Args:
            u (float): control input.

        Returns:
            float: h(x) derivative value
        """

        result = (
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
        return float(result.item())
