#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <functional>

void sub_callback(rclcpp::Node::SharedPtr node,
                  const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Subscribe: %d", msg->data);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_sub1_1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    std::function<void(const std_msgs::msg::Int32::SharedPtr)> fn =
        std::bind(sub_callback, node, std::placeholders::_1);

    auto sub = node->create_subscription<std_msgs::msg::Int32>(
        "topic_1_1", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
