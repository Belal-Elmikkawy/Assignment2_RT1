#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt.srv import GetVelocity  # Import our custom service
import threading
import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Publisher to drive the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service to provide velocity stats
        self.srv = self.create_service(GetVelocity, 'get_avg_velocity', self.get_avg_callback)

        # Storage for the last 5 velocities
        self.vel_history = []

        self.get_logger().info("Controller Node Started. Ready for input.")

    def publish_velocity(self, lin_x, ang_z):
        """
        Publishes the twist message and updates the history.
        """
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.publisher_.publish(msg)

        # Store data for the service requirement
        self.update_history(lin_x, ang_z)

    def update_history(self, lin, ang):
        """
        Keeps only the last 5 commands.
        """
        self.vel_history.append((lin, ang))

        # If we have more than 5, pop the oldest one
        if len(self.vel_history) > 5:
            self.vel_history.pop(0)

    def get_avg_callback(self, request, response):
        """
        Service logic: Calculates average of the stored history.
        """
        if not self.vel_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            self.get_logger().warn("Request received, but history is empty.")
            return response

        sum_lin = 0.0
        sum_ang = 0.0

        for v in self.vel_history:
            sum_lin += v[0]
            sum_ang += v[1]

        count = len(self.vel_history)
        response.avg_linear = sum_lin / count
        response.avg_angular = sum_ang / count

        self.get_logger().info(
            f"Service called. Calculated Avg Lin: {
                response.avg_linear:.2f}, Avg Ang: {
                response.avg_angular:.2f}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    # Run ROS communication in a separate thread so input() doesn't block it
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            print("\n--- Robot Controller ---")
            l_vel = input("Enter Linear Velocity (x): ")
            a_vel = input("Enter Angular Velocity (z): ")

            try:
                l_vel = float(l_vel)
                a_vel = float(a_vel)

                # Send the command
                node.publish_velocity(l_vel, a_vel)
                print(f"Sent: Linear={l_vel}, Angular={a_vel}")

            except ValueError:
                print("Invalid input! Please enter numbers.")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
