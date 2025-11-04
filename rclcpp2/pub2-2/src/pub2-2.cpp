#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

void callback(rclcpp::Node::SharedPtr node,
              rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub)
{
    static geometry_msgs::msg::Vector3 msg;
    std::cout << "x y z 실수값 3개를 입력하세요: ";
    std::cin >> msg.x >> msg.y >> msg.z;

    if (!std::cin) return;

    RCLCPP_INFO(node->get_logger(),
                "Publish: x=%f, y=%f, z=%f",
                msg.x, msg.y, msg.z);

    mypub->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mynode2");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("vector_topic", qos_profile);

    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
