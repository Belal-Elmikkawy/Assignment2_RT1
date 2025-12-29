#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment2_rt.msg import ObstacleInfo
from assignment2_rt.srv import SetThreshold
import math


class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')

        # 1. Variables
        self.safety_threshold = 1.0  # Default threshold in meters
        self.width = 0  # To store the width of the laser scan array

        # 2. Subscribers
        # Listen to the laser scanner to detect obstacles
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 3. Publishers
        # To publish safety commands (override user input)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # To publish the custom status message
        self.pub_info = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)

        # 4. Service
        # Allow changing the threshold dynamically
        self.create_service(SetThreshold, 'set_safety_threshold', self.set_threshold_callback)

        self.get_logger().info("Monitoring Node Started. Threshold: 1.0m")

    def set_threshold_callback(self, request, response):
        """
        Service to update the safety threshold.
        """
        self.safety_threshold = request.new_threshold
        response.success = True
        self.get_logger().info(f"Threshold updated to: {self.safety_threshold}m")
        return response

    def get_sector_name(self, index, total_len):
        """
        Helper function: Divides the laser array into 3 sectors to determine direction.
        """
        # We split the 180 (or 360) degree view into 3 chunks: Right, Front, Left
        one_third = total_len / 3

        if index < one_third:
            return "Right"
        elif index < 2 * one_third:
            return "Front"
        else:
            return "Left"

    def scan_callback(self, msg):
        """
        Main logic loop: Runs every time the laser sends data.
        """
        # Process Laser Data
        # Filter out invalid values (inf) which mean "too far to see"
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]

        if not valid_ranges:
            return  # No valid data, do nothing

        # Find the closest obstacle
        min_distance = min(valid_ranges)

        # Find the index of that minimum distance in the original array
        try:
            min_index = msg.ranges.index(min_distance)
            direction = self.get_sector_name(min_index, len(msg.ranges))
        except ValueError:
            direction = "Unknown"

        # Publish Custom Message (Requirement)
        info_msg = ObstacleInfo()
        info_msg.distance = float(min_distance)
        info_msg.direction = direction
        info_msg.threshold = float(self.safety_threshold)
        self.pub_info.publish(info_msg)

        # Check Safety (Requirement)
        if min_distance < self.safety_threshold:
            self.get_logger().warn(f"OBSTACLE DETECTED in {direction}! Moving Back...")

            # Create a Twist message to move BACKWARDS
            stop_msg = Twist()
            stop_msg.linear.x = -0.5  # Move back
            stop_msg.angular.z = 0.0

            # Publish it to /cmd_vel (This overrides the Controller Node's commands)
            self.pub_cmd.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
