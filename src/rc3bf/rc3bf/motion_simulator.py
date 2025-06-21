import numpy as np


class MotionSimulator:
    def __init__(self, initial_position=(0.0, 0.0, 0.0), dt=0.1):
        """
        Initialize the simulator.
        :param initial_position: Tuple (x, y, theta) representing the initial position and orientation.
        :param dt: Time step for the simulation.
        """
        self.x, self.y, self.theta = initial_position
        self.dt = dt

    def step(self, velocity: float, yaw_rate: float):
        """
        Simulate one time step of the robot's motion.
        :param velocity: Linear velocity (m/s).
        :param yaw_rate: Angular velocity (rad/s).
        """
        # Update orientation
        self.theta += yaw_rate * self.dt
        self.theta = (
            np.mod(self.theta + np.pi, 2 * np.pi) - np.pi
        )  # Normalize angle to [-pi, pi]

        # Update position
        self.x += velocity * np.cos(self.theta) * self.dt
        self.y += velocity * np.sin(self.theta) * self.dt

    def get_state(self) -> tuple[float, float, float]:
        """
        Get the current state of the robot.
        :return: Tuple (x, y, theta).
        """
        return self.x, self.y, self.theta


if __name__ == "__main__":
    # Example usage
    simulator = MotionSimulator(
        initial_position=(0.0, 0.0, 0.0), dt=0.1
    )

    # Simulate for 10 seconds with constant velocity and yaw rate
    velocity = 1.0  # m/s
    yaw_rate = 0.1  # rad/s
    for _ in range(100):  # 10 seconds with dt=0.1
        simulator.step(velocity, yaw_rate)
        print("State:", simulator.get_state())
