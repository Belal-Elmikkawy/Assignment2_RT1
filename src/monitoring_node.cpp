#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2_rt/msg/obstacle_info.hpp"
#include "assignment2_rt/srv/set_threshold.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

class MonitoringNode : public rclcpp::Node
{
public:
    MonitoringNode() : Node("monitoring_node"), safety_threshold_(1.0), reversing_(false)
    {
        // 1. Subscribers & Publishers
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MonitoringNode::scan_callback, this, std::placeholders::_1));
        
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_info_ = this->create_publisher<assignment2_rt::msg::ObstacleInfo>("/obstacle_info", 10);

        // 2. Service
        srv_threshold_ = this->create_service<assignment2_rt::srv::SetThreshold>(
            "set_safety_threshold",
            std::bind(&MonitoringNode::set_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Monitoring Node (C++) Started. Threshold: 1.0m");
    }

private:
    void set_threshold_callback(const std::shared_ptr<assignment2_rt::srv::SetThreshold::Request> request,
                                std::shared_ptr<assignment2_rt::srv::SetThreshold::Response> response)
    {
        safety_threshold_ = request->new_threshold;
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Threshold updated to: %.2fm", safety_threshold_);
    }

    std::string get_sector_name(size_t index, size_t total_len)
    {
        size_t one_third = total_len / 3;
        if (index < one_third) return "Right";
        else if (index < 2 * one_third) return "Front";
        else return "Left";
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum distance (ignoring inf/nan)
        float min_distance = std::numeric_limits<float>::infinity();
        size_t min_index = 0;
        bool found_valid = false;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (!std::isinf(r) && !std::isnan(r)) {
                if (r < min_distance) {
                    min_distance = r;
                    min_index = i;
                    found_valid = true;
                }
            }
        }

        if (!found_valid) return;

        std::string direction = get_sector_name(min_index, msg->ranges.size());

        // Publish Info
        auto info_msg = assignment2_rt::msg::ObstacleInfo();
        info_msg.distance = min_distance;
        info_msg.direction = direction;
        info_msg.threshold = safety_threshold_;
        pub_info_->publish(info_msg);

        // Safety Logic
        auto twist_msg = geometry_msgs::msg::Twist();

        if (min_distance < safety_threshold_) {
            RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED in %s! Moving Back...", direction.c_str());
            reversing_ = true;
            twist_msg.linear.x = -0.5;
            twist_msg.angular.z = 0.0;
            pub_cmd_->publish(twist_msg);
        }
        else if (reversing_) {
            // Obstacle cleared, stop backing up
            RCLCPP_INFO(this->get_logger(), "Safe distance restored. Stopping robot.");
            reversing_ = false;
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            pub_cmd_->publish(twist_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<assignment2_rt::msg::ObstacleInfo>::SharedPtr pub_info_;
    rclcpp::Service<assignment2_rt::srv::SetThreshold>::SharedPtr srv_threshold_;

    float safety_threshold_;
    bool reversing_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitoringNode>());
    rclcpp::shutdown();
    return 0;
}
