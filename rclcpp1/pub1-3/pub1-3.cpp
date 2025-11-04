#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <memory>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_pub1_3");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);

    geometry_msgs::msg::Twist msg;
    while (rclcpp::ok())
    {
        char key;
        std::cout << "키입력: ";
        std::cin >> key;

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        if (key == 'f')
            msg.linear.x = 1.0;
        else if (key == 'b')
            msg.linear.x = -1.0;
        else if (key == 'l')
            msg.angular.z = 1.0;
        else if (key == 'r')
            msg.angular.z = -1.0;
        else if (key == 'q')
            break;
        else
        {
            std::cout << "잘못된 입력입니다.\n";
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "Publish cmd_vel: linear=%.f angular=%.f", msg.linear.x, msg.angular.z);
        pub->publish(msg);
    }

    rclcpp::shutdown();
    return 0;
}

