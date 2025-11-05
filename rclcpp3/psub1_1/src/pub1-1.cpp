#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
#include <functional>
using namespace std::chrono_literals;

void timer_callback(rclcpp::Node::SharedPtr node,
                    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
{
    static int count = 0;
    std_msgs::msg::Int32 msg;
    msg.data = count++;
    RCLCPP_INFO(node->get_logger(), "Publish: %d", msg.data);
    pub->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_pub1_1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<std_msgs::msg::Int32>("topic_1_1", qos_profile);

    // 100ms마다 콜백 실행
    std::function<void()> fn = std::bind(timer_callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
