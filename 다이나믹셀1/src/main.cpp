#include "rclcpp/rclcpp.hpp"
#include "dxl3/dxl.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dxl3");

    Dxl mx;  // Dynamixel 제어 객체 생성

    if(!mx.open()) {
        RCLCPP_INFO(node->get_logger(), "Dynamixel open failed");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::WallRate loop_rate(10);  // 0.1초(10Hz)

    int cmd = 0;          // 현재 속도 명령 (rpm)
    int delta = 10;       // 속도 변화량 (10rpm씩 증가/감소)

    while(rclcpp::ok())
    {
        // --- 속도 증가/감소 로직 ---
        cmd += delta;

        if(cmd >= 100) {      // +100rpm 도달 → 감소 시작
            delta = -10;
        }
        else if(cmd <= -100) { // -100rpm 도달 → 증가 시작
            delta = 10;
        }

        // 왼쪽/오른쪽 동일 속도 명령
        mx.setVelocity(cmd, cmd);

        RCLCPP_INFO(node->get_logger(), "cmd rpm = %d", cmd);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

