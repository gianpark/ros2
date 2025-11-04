#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>

int main(int argc, char *argv[])
{
    static int x=0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1", qos_profile);
    std_msgs::msg::Int32 message; 
    rclcpp::WallRate loop_rate(1.0);
    while (rclcpp::ok())
    {
        message.data = x++;
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);
        mypub->publish(message);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

