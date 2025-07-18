import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

import numpy as np

from .motion_simulator import MotionSimulator
from .base_controller import HolonomicMobileRobotController

# from .safety_filter import SafetyFilter
from .rc3bf_safety_filter import RobustSafetyFilter


class Robot(Node):
    def __init__(self):
        super().__init__("robot_node")
        self.get_logger().info("Robot node has been started.")
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            timer_period, self.timer_callback
        )

        # Read parameters
        self.declare_parameter("sampling_time", 0.1)
        sampling_time = self.get_parameter("sampling_time").value

        self.declare_parameter("initial_position", [0.0, 0.0, 0.0])
        initial_position = self.get_parameter(
            "initial_position"
        ).value

        self.declare_parameter("kp_linear", 1.0)
        kp_linear = self.get_parameter("kp_linear").value

        self.declare_parameter("kp_angular", 0.1)
        kp_angular = self.get_parameter("kp_angular").value

        self.declare_parameter("target_position", [0.0, 0.0])
        self.target_position = self.get_parameter(
            "target_position"
        ).value

        self.declare_parameter("robot_radius", 1.0)
        robot_radius = self.get_parameter("robot_radius").value

        self.declare_parameter(
            "robot_obstacle_name", "robot_obstacle"
        )
        robot_obstacle_name = self.get_parameter(
            "robot_obstacle_name"
        ).value

        self.declare_parameter("use_cbf", True)
        self.use_cbf = self.get_parameter("use_cbf").value
        self.get_logger().info(f": h={self.use_cbf}")

        # Internal Variables
        self.obstacle_pose = np.array([0.0, 0.0])  # [x, y]
        self.obstacle_velocity = [0.0, 0.0]  # [vx, vy]

        # Initialization code
        self.motion_simulator = MotionSimulator(
            initial_position=initial_position, dt=sampling_time
        )
        self.controller = HolonomicMobileRobotController(
            kp_linear=kp_linear, kp_angular=kp_angular
        )
        self.safety_filter = RobustSafetyFilter(
            obstacle_radius_r=robot_radius,  # rad/s
            epsilon=0.01,  # m/s^2
        )

        # Publisher for pose
        self.pose_publisher = self.create_publisher(
            PoseStamped, "robot_pose", 10
        )
        self.frame_id = "map"

        self.publisher_v = self.create_publisher(Float32, "v", 10)
        self.publisher_omega = self.create_publisher(
            Float32, "omega", 10
        )

        self.publisher_h = self.create_publisher(Float32, "h", 10)
        self.publisher_hp = self.create_publisher(Float32, "hp", 10)

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            "/" + robot_obstacle_name + "/robot_pose",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        x, y, theta = self.motion_simulator.get_state()
        self.publish_pose(x, y, theta)

        v, yr = self.controller.compute_control(
            (x, y, theta), self.target_position
        )
        # v = np.minimum(v, 1.0)  # Limit linear speed to 1.0 m/s

        robot_pose = np.array([x, y, theta])
        # obstacle_pose = np.array([20.0, 20.0])
        # obstacle_vel = np.array([0.0, 0.0])

        if self.use_cbf:
            yr_safe = self.safety_filter.run_filter(
                robot_pose,
                self.obstacle_pose,
                self.obstacle_velocity,
                yr,
                v,
            )
            if yr_safe is None:
                self.get_logger().warn(
                    "Safety filter returned None, setting yr to 0.0"
                )
                yr_safe = 0.0
                v = 0.0
            else:
                yr = yr_safe

                msg_h = Float32()
                msg_h.data = float(self.safety_filter.h())
                self.publisher_h.publish(msg_h)

                msg_hp = Float32()
                msg_hp.data = float(
                    float(self.safety_filter.h_prim(0.0))
                )
                self.publisher_hp.publish(msg_hp)

        self.motion_simulator.step(v, yr)

        if not np.isnan(v):
            self.get_logger().info(
                f"Control: h={self.safety_filter.h()}, h_prim={self.safety_filter.h_prim(0)}"
            )

            msg_v = Float32()
            msg_v.data = float(v)
            self.publisher_v.publish(msg_v)

        if not np.isnan(yr):
            self.get_logger().info(f"Control: u_safe={yr}")

            msg_omega = Float32()
            msg_omega.data = float(yr)
            self.publisher_omega.publish(msg_omega)

    def listener_callback(self, msg):
        self.obstacle_velocity[0] = (
            msg.pose.position.x - self.obstacle_pose[0]
        ) / 0.01
        self.obstacle_velocity[1] = (
            msg.pose.position.y - self.obstacle_pose[1]
        ) / 0.01

        self.obstacle_pose[0] = msg.pose.position.x
        self.obstacle_pose[1] = msg.pose.position.y
        # self.get_logger().info(f"Obstacle {self.obstacle_pose}")

    def move(self, direction):
        self.get_logger().info(f"Moving {direction}")

    def publish_pose(self, x, y, theta):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0

        # R = matrix_from_euler_xyz([0, 0, theta])
        # q = quaternion_from_matrix(R)

        # pose_msg.pose.orientation.x = q[0]
        # pose_msg.pose.orientation.y = q[1]
        # pose_msg.pose.orientation.z = q[2]
        # pose_msg.pose.orientation.w = q[3]

        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
