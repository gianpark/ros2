#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <vector>
#include <memory>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::Float64MultiArray>("topic_pub1_2", qos_profile);
    std_msgs::msg::Float64MultiArray message;
    rclcpp::WallRate loop_rate(1.0);
    while (rclcpp::ok())
    {
        double a, b, c;
        std::cout << "실수값 3개를 입력하세요";
        std::cin >> a >> b >> c;
        message.data.push_back(a);
        message.data.push_back(b);
        message.data.push_back(c);
        RCLCPP_INFO(node->get_logger(), "Publish: [%f, %f, %f]", a, b, c);
        mypub->publish(message);

        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

