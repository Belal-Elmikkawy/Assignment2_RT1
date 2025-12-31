#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt.srv import GetVelocity  # Import our custom service definition
import threading
import sys


class ControllerNode(Node):
    def __init__(self):
        # Initialize the node with the name 'controller_node'
        super().__init__('controller_node')

        # --- KEY VARIABLES ---
        # Publisher: Sends velocity commands to the simulation
        # Topic: '/cmd_vel', Message Type: Twist, Queue Size: 10
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service Server: Responds to requests for velocity statistics
        # Service Name: 'get_avg_velocity', Type: GetVelocity
        self.srv = self.create_service(GetVelocity, 'get_avg_velocity', self.get_avg_callback)

        # History List: Stores the last 5 velocity commands
        # Format: [(lin1, ang1), (lin2, ang2), ...]
        self.vel_history = []

        self.get_logger().info("Controller Node Started. Ready for input.")

    def publish_velocity(self, lin_x, ang_z):
        """
        Function to send a drive command to the robot.
        Args:
            lin_x (float): Linear velocity (forward/backward)
            ang_z (float): Angular velocity (turning)
        """
        # Create the message object
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        
        # Publish the message to /cmd_vel topic
        self.publisher_.publish(msg)

        # Save this command to our history list for the service
        self.update_history(lin_x, ang_z)

    def update_history(self, lin, ang):
        """
        Helper function to manage the size of the history list.
        Implements a FIFO (First-In, First-Out) queue of size 5.
        """
        # Add new command tuple to the end of the list
        self.vel_history.append((lin, ang))

        # If list exceeds 5 items, remove the oldest item (index 0)
        if len(self.vel_history) > 5:
            self.vel_history.pop(0)

    def get_avg_callback(self, request, response):
        """
        Service Callback: Triggered when another node calls '/get_avg_velocity'.
        Calculates the average of linear and angular velocities in history.
        """
        # Safety Check: If history is empty, return 0.0 to avoid division by zero error
        if not self.vel_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            self.get_logger().warn("Request received, but history is empty.")
            return response

        # Initialize sums
        sum_lin = 0.0
        sum_ang = 0.0

        # Loop through stored history to calculate sums
        for v in self.vel_history:
            sum_lin += v[0]  # Add linear velocity
            sum_ang += v[1]  # Add angular velocity

        # Calculate Averages
        count = len(self.vel_history)
        response.avg_linear = sum_lin / count
        response.avg_angular = sum_ang / count

        # Log the result for debugging
        self.get_logger().info(
            f"Service called. Calculated Avg Lin: {response.avg_linear:.2f}, Avg Ang: {response.avg_angular:.2f}")
        
        # Return the filled response object to the client
        return response


def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    node = ControllerNode()

    # --- THREADING EXPLANATION ---
    # The input() function is 'blocking', meaning code execution stops waiting for the user.
    # ROS 2 needs to keep running (spinning) to listen for Service requests in the background.
    # Therefore, we run rclpy.spin() in a separate thread.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Main Control Loop: Keeps asking for user input until the node is shut down
        while rclpy.ok():
            print("\n---------------------------------")
            print("--- Robot Controller Interface ---")
            l_vel = input("Enter Linear Velocity (x): ")
            a_vel = input("Enter Angular Velocity (z): ")

            try:
                # Convert string input to float
                l_vel = float(l_vel)
                a_vel = float(a_vel)

                # Send command to robot
                node.publish_velocity(l_vel, a_vel)
                print(f"Sent: Linear={l_vel}, Angular={a_vel}")

            except ValueError:
                # Handle cases where user types text instead of numbers
                print("Invalid input! Please enter numeric values.")

    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\nExiting...")
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
