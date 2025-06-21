import math


class HolonomicMobileRobotController:
    def __init__(self, kp_linear=1.0, kp_angular=1.0):
        """
        Initialize the controller with proportional gains for linear and angular control.
        :param kp_linear: Proportional gain for linear velocity control.
        :param kp_angular: Proportional gain for angular velocity control.
        """
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular

    def compute_control(self, current_state, desired_position):
        """
        Compute the control inputs (velocity, yaw rate) for the robot.
        :param current_state: Tuple (x, y, yaw) representing the current state of the robot.
        :param desired_position: Tuple (x_d, y_d) representing the desired position of the robot.
        :return: Tuple (velocity, yaw_rate) as control inputs.
        """
        x, y, yaw = current_state
        x_d, y_d = desired_position

        # Compute the error in position
        error_x = x_d - x
        error_y = y_d - y

        # Compute the distance to the target
        distance = math.sqrt(error_x**2 + error_y**2)

        # Compute the desired yaw angle
        desired_yaw = math.atan2(error_y, error_x)

        # Compute the yaw error
        yaw_error = desired_yaw - yaw
        yaw_error = math.atan2(
            math.sin(yaw_error), math.cos(yaw_error)
        )  # Normalize to [-pi, pi]

        # Compute control inputs
        velocity = self.kp_linear * distance
        yaw_rate = self.kp_angular * yaw_error

        return velocity, yaw_rate
