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
    : Node("line_detector"), prev_center_x_(-1.0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video", rclcpp::QoS(10),
            std::bind(&LineDetector::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LineDetector Node Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start = std::chrono::steady_clock::now();

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        if (frame.empty()) return;

        int roi_width = 640;
        int roi_height = 90;

        int roi_x = 0;
        int roi_y = frame.rows - roi_height;
        if (roi_y < 0) roi_y = 0;

        cv::Rect roi_rect(roi_x, roi_y,
                          std::min(roi_width, frame.cols - roi_x),
                          std::min(roi_height, frame.rows - roi_y));
        cv::Mat roi = frame(roi_rect).clone();

        // 밝기 보정
        const double target_mean = 128.0;
        cv::Scalar mean_scalar = cv::mean(roi);
        double current_mean = (mean_scalar[0] + mean_scalar[1] + mean_scalar[2]) / 3.0;
        double shift = target_mean - current_mean;
        roi.convertTo(roi, -1, 0.6, shift);

        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // 연결요소 분석
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

        std::vector<cv::Rect> candidates;
        std::vector<int> centers_x;

        for (int i = 1; i < n; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int y = stats.at<int>(i, cv::CC_STAT_TOP);

            if (area > 30 && area < 20000) {
                candidates.emplace_back(x, y, w, h);
                centers_x.push_back(x + w/2);
            }
        }

        if (prev_center_x_ < 0) prev_center_x_ = frame.cols / 2.0;

        int selected = -1;
        double best_dist = 1e9;

        for (size_t i = 0; i < centers_x.size(); i++) {
            double dist = std::abs((roi_x + centers_x[i]) - prev_center_x_);
            if (dist < best_dist) {
                best_dist = dist;
                selected = i;
            }
        }

        double cx_final = prev_center_x_;

        for (size_t i = 0; i < candidates.size(); i++) {
            auto &r = candidates[i];
            if ((int)i == selected) {
                cv::rectangle(display, r, cv::Scalar(0,0,255), 2);
                int cx = r.x + r.width / 2;
                int cy = r.y + r.height / 2;
                cv::circle(display, {cx, cy}, 4, {0,0,255}, -1);
                cx_final = roi_x + cx;
                prev_center_x_ = cx_final;
            } else {
                cv::rectangle(display, r, cv::Scalar(255,0,0), 2);
            }
        }

        if (selected == -1) {
            int cx = (int)prev_center_x_ - roi_x;
            if (cx >= 0 && cx < display.cols)
                cv::circle(display, {cx, display.rows/2}, 4, {0,0,255}, -1);
        }

        double error = frame.cols/2.0 - cx_final;

        auto end = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "error:%d, time: %.4f sec",
                    (int)std::round(error), t);

        // === 🆕 Original Video Display 추가 ===
        cv::imshow("Original Frame", frame);

        // 기존 Overlay 출력 그대로 유지
        cv::imshow("Binary with Overlay", display);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    double prev_center_x_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetector>());
    rclcpp::shutdown();
    return 0;
}

