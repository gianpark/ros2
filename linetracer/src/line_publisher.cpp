#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() : Node("video_publisher")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("video", 10);

        timer_ = this->create_wall_timer(
            30ms,
            std::bind(&VideoPublisher::timer_callback, this)
        );

        std::string video_path = "/home/linux/ros2_ws/simulation/5_lt_cw_100rpm_out.mp4";
        cap_.open(video_path);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
        }
    }

private:
    void timer_callback()
    {
        if (!cap_.isOpened()) return;

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video ended");
            rclcpp::shutdown();
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}

