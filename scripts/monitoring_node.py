#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment2_rt.msg import ObstacleInfo   # Import from YOUR package
from assignment2_rt.srv import SetThreshold   # Import from YOUR package
import math


class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')

        self.safety_threshold = 1.0
        self.last_cmd_vel = Twist()

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_info = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)
        self.create_service(SetThreshold, 'set_safety_threshold', self.set_threshold_callback)

        self.get_logger().info("Monitoring Node Started.")

    def vel_callback(self, msg):
        self.last_cmd_vel = msg

    def set_threshold_callback(self, request, response):
        self.safety_threshold = request.new_threshold
        response.success = True
        return response

    def get_sector_name(self, index, total_len):
        one_third = total_len / 3
        if index < one_third:
            return "Right"
        elif index < 2 * one_third:
            return "Front"
        else:
            return "Left"

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)
        try:
            min_index = msg.ranges.index(min_distance)
            direction = self.get_sector_name(min_index, len(msg.ranges))
        except ValueError:
            direction = "Unknown"

        # Dashboard Display
        print(f"\033[2J\033[H", end="")
        print("=" * 40)
        print("       MONITORING DASHBOARD       ")
        print("=" * 40)
        print(
            f" COMMAND: Lin={
                self.last_cmd_vel.linear.x:.2f}, Ang={
                self.last_cmd_vel.angular.z:.2f}")
        print("-" * 40)
        print(f" SENSOR:  Min Dist={min_distance:.2f}m ({direction})")
        print(f" LIMIT:   Threshold={self.safety_threshold:.2f}m")
        print("-" * 40)

        if min_distance < self.safety_threshold:
            print(f" STATUS:  [!!! COLLISION RISK !!!]")
            stop_msg = Twist()
            if direction == "Front":
                stop_msg.linear.x = -1.0
            elif direction == "Left":
                stop_msg.angular.z = -1.0
            elif direction == "Right":
                stop_msg.angular.z = 1.0
            else:
                stop_msg.linear.x = 1.0
            self.pub_cmd.publish(stop_msg)
        else:
            print(f" STATUS:  [ OK ] Safe")
        print("=" * 40)

        info_msg = ObstacleInfo()
        info_msg.distance = float(min_distance)
        info_msg.direction = direction
        info_msg.threshold = float(self.safety_threshold)
        self.pub_info.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
