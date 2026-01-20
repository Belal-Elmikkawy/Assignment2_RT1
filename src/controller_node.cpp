#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2_rt/srv/get_velocity.hpp"

#include <iostream>
#include <vector>
#include <numeric>
#include <thread>
#include <deque>
#include <limits>
#include <mutex>

// Using std::deque for the history buffer (FIFO)
struct VelCommand {
    float linear;
    float angular;
};

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // 1. Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 2. Service Server
        service_ = this->create_service<assignment2_rt::srv::GetVelocity>(
            "get_avg_velocity",
            std::bind(&ControllerNode::get_avg_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Controller Node (C++) Started. Ready for input.");
    }

    void publish_velocity(float lin_x, float ang_z)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = lin_x;
        msg.angular.z = ang_z;
        publisher_->publish(msg);

        update_history(lin_x, ang_z);
    }

private:
    void update_history(float lin, float ang)
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        vel_history_.push_back({lin, ang});
        if (vel_history_.size() > 5) {
            vel_history_.pop_front();
        }
    }

    void get_avg_callback(const std::shared_ptr<assignment2_rt::srv::GetVelocity::Request> request,
                          std::shared_ptr<assignment2_rt::srv::GetVelocity::Response> response)
    {
        (void)request; // Avoid unused parameter warning

        std::lock_guard<std::mutex> lock(history_mutex_);
        if (vel_history_.empty()) {
            response->avg_linear = 0.0;
            response->avg_angular = 0.0;
            RCLCPP_WARN(this->get_logger(), "Request received, but history is empty.");
            return;
        }

        float sum_lin = 0.0;
        float sum_ang = 0.0;

        for (const auto& cmd : vel_history_) {
            sum_lin += cmd.linear;
            sum_ang += cmd.angular;
        }

        response->avg_linear = sum_lin / vel_history_.size();
        response->avg_angular = sum_ang / vel_history_.size();

        RCLCPP_INFO(this->get_logger(), "Service called. Avg Lin: %.2f, Avg Ang: %.2f",
                    response->avg_linear, response->avg_angular);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<assignment2_rt::srv::GetVelocity>::SharedPtr service_;
    std::deque<VelCommand> vel_history_;
    std::mutex history_mutex_;
};

// --- INPUT LOOP FUNCTION ---
void user_input_loop(std::shared_ptr<ControllerNode> node)
{
    float l_vel, a_vel;
    while (rclcpp::ok()) {
        std::cout << "\n---------------------------------" << std::endl;
        std::cout << "--- Robot Controller Interface (C++) ---" << std::endl;
        std::cout << "Enter Linear Velocity (x): ";
        if (!(std::cin >> l_vel)) {
            std::cin.clear(); std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue; // Handle bad input
        }
        std::cout << "Enter Angular Velocity (z): ";
        if (!(std::cin >> a_vel)) {
            std::cin.clear(); std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        node->publish_velocity(l_vel, a_vel);
        std::cout << "Sent: Linear=" << l_vel << ", Angular=" << a_vel << std::endl;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();

    // Run the user input loop in a separate thread so ROS callbacks (service) can run in main
    std::thread input_thread(user_input_loop, node);

    rclcpp::spin(node);

    // Cleanup
    if (input_thread.joinable()) input_thread.detach();
    rclcpp::shutdown();
    return 0;
}
