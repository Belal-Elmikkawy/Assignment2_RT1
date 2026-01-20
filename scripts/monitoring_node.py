#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment2_rt.msg import ObstacleInfo  # Custom message for reporting
from assignment2_rt.srv import SetThreshold  # Custom service for settings
import math


class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')

        # --- KEY VARIABLES ---
        # Safety Threshold: Distance (meters) below which safety override triggers
        self.safety_threshold = 1.0
        self.width = 0
        self.reversing = False

        # --- SUBSCRIBERS ---
        # Listen to the Laser Scanner to get distance data
        # Topic: '/scan', Message: LaserScan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- PUBLISHERS ---
        # Command Publisher: To stop/move robot when unsafe (overrides Controller Node)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # Info Publisher: Publishes the custom message required by the assignment
        self.pub_info = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)

        # --- SERVICES ---
        # Allows dynamic changing of the safety threshold
        self.create_service(SetThreshold, 'set_safety_threshold', self.set_threshold_callback)

        self.get_logger().info("Monitoring Node Started. Threshold: 1.0m")

    def set_threshold_callback(self, request, response):
        """
        Service Callback: Updates the safety distance threshold.
        """
        self.safety_threshold = request.new_threshold
        response.success = True
        self.get_logger().info(f"Threshold updated to: {self.safety_threshold}m")
        return response

    def get_sector_name(self, index, total_len):
        """
        Helper Function: Determines the direction of the obstacle based on the index
        of the laser ray in the array.
        Logic: Splits the field of view into 3 sectors (Right, Front, Left).
        """
        one_third = total_len / 3

        if index < one_third:
            return "Right"
        elif index < 2 * one_third:
            return "Front"
        else:
            return "Left"

    def scan_callback(self, msg):
        """
        Main Logic Loop: Runs every time the robot receives laser data.
        """
        # DATA FILTERING ---
        # Filter out 'inf' (infinity) and 'nan' (not a number) values.
        # These usually mean the laser didn't hit anything within range.
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]

        # If no valid data is found, assume we are safe or blind, and return.
        if not valid_ranges:
            return

        # FIND CLOSEST OBSTACLE ---
        # Get the minimum distance value from the filtered list
        min_distance = min(valid_ranges)

        # Find which index (ray) this minimum distance corresponds to
        # This is needed to calculate the 'direction'
        try:
            min_index = msg.ranges.index(min_distance)
            direction = self.get_sector_name(min_index, len(msg.ranges))
        except ValueError:
            direction = "Unknown"

        # PUBLISH STATUS (Assignment Requirement) ---
        # Create and populate the custom message
        info_msg = ObstacleInfo()
        info_msg.distance = float(min_distance)
        info_msg.direction = direction
        info_msg.threshold = float(self.safety_threshold)
        self.pub_info.publish(info_msg)

        # SAFETY CHECK (Assignment Requirement) ---
        # If the closest object is closer than our threshold...
        if min_distance < self.safety_threshold:
            self.get_logger().warn(f"OBSTACLE DETECTED in {direction}! Moving Back...")
            self.reversing = True  # Set flag that we are in reversing state

            # Create a Twist message to move BACKWARDS
            stop_msg = Twist()
            stop_msg.linear.x = -0.5  # Negative X moves the robot backward
            stop_msg.angular.z = 0.0

            # Publish this command to /cmd_vel.
            self.pub_cmd.publish(stop_msg)

        elif self.reversing:
            # We are now SAFE (min_distance >= threshold), but we were reversing.
            # We need to stop the robot so it doesn't keep going backwards forever.
            self.get_logger().info("Safe distance restored. Stopping robot.")
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.pub_cmd.publish(stop_msg)

            self.reversing = False  # Reset flag


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()

    try:
        # Spin keeps the node active, processing callbacks (scan, service)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
