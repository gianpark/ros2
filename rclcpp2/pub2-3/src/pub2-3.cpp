#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

void callback(rclcpp::Node::SharedPtr node,
              rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0; 
    msg.angular.z = 1.0; 

    RCLCPP_INFO(node->get_logger(), "Publishing: linear=%f angular=%f",
                msg.linear.x, msg.angular.z);

    pub->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mynode3");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);
    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(500ms, fn); 

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
