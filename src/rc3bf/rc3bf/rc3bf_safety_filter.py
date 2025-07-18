import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm


class RobustSafetyFilter:
    def __init__(
        self,
        obstacle_radius_r: float,
        epsilon: float = 1e-6,
    ):
        if obstacle_radius_r <= 0.0:
            raise ValueError(
                "Obstacle radius must be greater than zero for stability."
            )
        if epsilon <= 0.0:
            raise ValueError(
                "Epsilon must be greater than zero for numerical stability."
            )

        self.r = obstacle_radius_r
        self.epsilon = epsilon

        self.u_nom = 0.0
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.obstacle_position = np.array([0.0, 0.0])
        self.obstacle_velocity = np.array([0.0, 0.0])

        self.p = np.array([0.0, 0.0])
        self.v = np.array([0.0, 0.0])
        self.a = np.array([0.0, 0.0])

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
            val = self.h_prim(u) + alpha * self.h() - norm.ppf(0.95) * sigma
            # Zwracamy tablicę 1D, bo scipy minimize wymaga tego formatu
            return np.array([val])

        cons = [
            {
                "type": "ineq",
                "fun": constraint,
            }
        ]

        self.u_nom = u_nominal

        result = minimize(
            self.objective_function,
            np.array([self.u_nom]),  # przekazujemy 1D array jako punkt startowy
            constraints=cons,
            method='SLSQP',
        )

        # Bezpiecznie wyciągamy wynik
        if result.success:
            u_safe = result.x[0]  # z 1-elementowego array bierzemy float
        else:
            # W razie niepowodzenia, daj nominalną wartość
            u_safe = self.u_nom

        self._calculate_relative_vectors(u_safe)
        return u_safe

    def objective_function(self, u):
        # u może być array (np. 1D array), więc u[0]
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
            np.dot(self.p, self.v)
            + np.linalg.norm(self.v)
            * np.sqrt(max(np.linalg.norm(self.p) ** 2 - self.r ** 2, 1e-6))
        )

    def h_prim(self, u: float) -> float:
        return float(
            np.dot(self.v, self.v)
            + np.dot(self.p, self.a)
            + np.dot(self.v, self.a)
            * np.sqrt(max(np.linalg.norm(self.p) ** 2 - self.r ** 2, 1e-6))
            / (np.linalg.norm(self.p) + self.epsilon)
            + np.dot(self.p, self.v)
            * np.linalg.norm(self.v)
            / np.sqrt(max(np.linalg.norm(self.p) ** 2 - self.r ** 2, 1e-6))
        )

    def sigma_h(self) -> float:
        delta = np.sqrt(max(np.linalg.norm(self.p) ** 2 - self.r ** 2, 1e-6))

        # Poprawiona implementacja - wektory gradientów
        nabla_pr_h = self.v + (np.linalg.norm(self.v) / delta) * self.p
        nabla_vr_h = self.p + (delta / np.linalg.norm(self.v)) * self.v

        # Druga pochodna lub poprawka gradientów
        # Z uwagi na oryginalny kod, uprościłem dla przykładu:
        nabla_pr_hp = (
            (np.linalg.norm(self.v) / delta) * self.v
            - (np.linalg.norm(self.v) * np.dot(self.p, self.v) * self.p) / delta**3
        )
        nabla_vr_hp = (
            2 * self.v
            + (np.linalg.norm(self.v) * self.p + np.dot(self.p, self.v) * self.v
                / np.linalg.norm(self.v))
            / delta
        )

        sigma_p = np.array([[0.1, 0.0], [0.0, 0.1]])
        sigma_v = np.array([[0.1, 0.0], [0.0, 0.1]])

        sp = nabla_pr_h + nabla_pr_hp
        sv = nabla_vr_h + nabla_vr_hp

        # Wynik musi być skalarem
        s = sp.T @ sigma_p @ sp + sv.T @ sigma_v @ sv
        s_scalar = float(s)  # wymuszamy float, a nie tablicę
        return s_scalar
