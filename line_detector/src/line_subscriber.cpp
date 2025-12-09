#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

class LineDetector : public rclcpp::Node
{
public:
    LineDetector()
    : Node("line_detector"), prev_center_x_(-1.0), lost_count_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video", rclcpp::QoS(10),
            std::bind(&LineDetector::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LineDetector Node Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start = std::chrono::steady_clock::now();   // ⬅ 추가: 시작 시각
        cv::Mat frame;

        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }
        if (frame.empty()) return;

        int frame_w = frame.cols;
        int frame_h = frame.rows;

        // === ROI ===
        int roi_width = 640;
        int roi_height = 90;
        int roi_x = 0;
        int roi_y = frame_h - roi_height;
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);
        cv::Mat roi = frame(roi_rect).clone();

        // 밝기 보정
        const double target_mean = 140.0;
        double shift = target_mean - cv::mean(roi)[0];
        roi.convertTo(roi, -1, 0.7, shift);

        // 이진화
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

        // 표시 영상
        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // 윤곽
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (prev_center_x_ < 0)
            prev_center_x_ = frame_w / 3.0;

        double best_center_x = prev_center_x_;
        double min_dist = 1e9;
        cv::Rect best_rect;
        bool found = false;

        for (const auto &cnt : contours)
        {
            cv::Rect rect = cv::boundingRect(cnt);
            double area = rect.area();
            int cx = rect.x + rect.width / 2;

            cv::rectangle(display, rect, cv::Scalar(255, 0, 0), 2);

            // 필터 조건
            if (area < 60) continue;
            if (rect.width < 3 || rect.height < 5) continue;
            if ((float)rect.height / rect.width < 0.10f) continue;

            double candidate_x = roi_rect.x + cx;
            double dist = std::abs(candidate_x - prev_center_x_);

            if (dist < min_dist) {
                min_dist = dist;
                best_center_x = candidate_x;
                best_rect = rect;
                found = true;
            }
        }

        const double max_jump = 160.0;
        const int max_lost = 5;
        const double max_speed = 12.0;
        const double reappear_speed = 6.0;

        if (found && min_dist < max_jump)
        {
            lost_count_ = 0;

            cv::rectangle(display, best_rect, cv::Scalar(0, 0, 255), 2);
            cv::circle(display,
                cv::Point(best_center_x - roi_rect.x,
                        best_rect.y + best_rect.height / 2),
                6, cv::Scalar(0, 0, 255), -1);

            double target = best_center_x;
            double dx = target - prev_center_x_;
            double limit = (lost_count_ > 0) ? reappear_speed : max_speed;

            if (std::abs(dx) > limit)
                target = prev_center_x_ + (dx > 0 ? limit : -limit);

            double alpha = 0.3;
            prev_center_x_ = prev_center_x_ * (1.0 - alpha) + target * alpha;
        }
        else
        {
            lost_count_++;
            if (lost_count_ > max_lost)
                lost_count_ = max_lost;

            double target_x = roi_rect.x + (roi_rect.width * 1.0 / 4.0);

            prev_center_x_ = prev_center_x_ * 0.85 + target_x * 0.15;

            cv::circle(display,
                cv::Point(prev_center_x_ - roi_rect.x, roi_rect.height - 10),
                6, cv::Scalar(0, 0, 255), -1);
        }


        double error = frame_w / 2.0 - prev_center_x_;

        // === ⬅ 추가: 시간 측정 ===
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double sec = elapsed.count();

        // === ⬅ 출력 형식: error:-10, time:0.30sec ===
        RCLCPP_INFO(this->get_logger(),
            "error:%d, time:%.4f sec",
            (int)error, sec);


        cv::imshow("Original Frame", frame);
        cv::imshow("Binary with Overlay", display);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    double prev_center_x_;
    int lost_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetector>());
    rclcpp::shutdown();
    return 0;
}
