#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Required for reading position
from assignment2_rt.srv import GetVelocity  # Import from YOUR package
import threading


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribe to /odom to track position
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(GetVelocity, 'get_avg_velocity', self.get_avg_callback)

        self.current_twist = Twist()
        self.vel_history = []
        self.robot_x = 0.0  # Track X position

        # --- LIMITS ---
        self.LIMIT_X_MAX = 3.0
        self.LIMIT_X_MIN = -3.0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f"Controller Started. Constraints: {
                self.LIMIT_X_MIN} < X < {
                self.LIMIT_X_MAX}")

    def odom_callback(self, msg):
        """Updates the robot's current X position."""
        self.robot_x = msg.pose.pose.position.x

    def timer_callback(self):
        """Checks limits and publishes velocity."""
        safe_cmd = Twist()
        safe_cmd.linear.x = self.current_twist.linear.x
        safe_cmd.angular.z = self.current_twist.angular.z

        # --- LIMIT LOGIC ---
        # 1. If at MAX limit (3.0) AND trying to go forward -> STOP
        if self.robot_x >= self.LIMIT_X_MAX and safe_cmd.linear.x > 0:
            self.get_logger().warn(
                f"LIMIT REACHED (X={
                    self.robot_x:.2f})! Stopping forward.",
                throttle_duration_sec=2)
            safe_cmd.linear.x = 0.0

        # 2. If at MIN limit (-3.0) AND trying to go backward -> STOP
        elif self.robot_x <= self.LIMIT_X_MIN and safe_cmd.linear.x < 0:
            self.get_logger().warn(
                f"LIMIT REACHED (X={
                    self.robot_x:.2f})! Stopping backward.",
                throttle_duration_sec=2)
            safe_cmd.linear.x = 0.0

        self.publisher_.publish(safe_cmd)

    def set_velocity(self, lin_x, ang_z):
        self.current_twist.linear.x = float(lin_x)
        self.current_twist.angular.z = float(ang_z)

        self.vel_history.append((lin_x, ang_z))
        if len(self.vel_history) > 5:
            self.vel_history.pop(0)

    def get_avg_callback(self, request, response):
        if not self.vel_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            return response

        sum_lin = sum(v[0] for v in self.vel_history)
        sum_ang = sum(v[1] for v in self.vel_history)

        response.avg_linear = sum_lin / len(self.vel_history)
        response.avg_angular = sum_ang / len(self.vel_history)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            print("\n" + "=" * 40)
            print(f" CONTROLLER INTERFACE")
            print(f" Status: X = {node.robot_x:.2f}")
            print(f" Limits: [{node.LIMIT_X_MIN}, {node.LIMIT_X_MAX}]")
            print("=" * 40)
            l_input = input("Enter Linear Velocity (x): ")
            a_input = input("Enter Angular Velocity (z): ")
            try:
                l_vel = float(l_input)
                a_vel = float(a_input)
                node.set_velocity(l_vel, a_vel)
                print(f"Target Set: v={l_vel}, w={a_vel}")
            except ValueError:
                print("Invalid Input.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
