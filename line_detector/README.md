결과 동영상 : https://youtu.be/ZymWpI6bNIE

ROS2 기반 라인 검출 실습 보고서
1. 제목

ROS2 기반 라인트레이싱 영상 처리 실습 보고서

2. 작성자

이름: 박기안

학번: 2001186

작성일: 2025/12/09

3. 목적

ROS2와 OpenCV를 활용하여 동영상에서 라인을 검출하고, 라인 중심과 화면 중심의 오차를 계산한다.

본 실습을 통해 라인 검출 알고리즘의 동작 원리를 이해하고, 실시간 영상 처리 성능을 평가하며, 자율주행 로봇 라인트레이싱 알고리즘 개발에 활용 가능한 기반을 마련한다.

4. 실험 환경

하드웨어: PC (CPU: AMD Ryzen 5 5500U, RAM: 16GB, GPU: 없음)

소프트웨어: Ubuntu 22.04, ROS2 Humble, OpenCV 4.8.1, C++17

사용 패키지: rclcpp, sensor_msgs, cv_bridge

영상 파일 경로: /home/linux/ros2_ws/simulation/7_lt_ccw_100rpm_in.mp4

5. 코드 설명 및 처리 과정
5.1 영상 퍼블리시 노드 (VideoPublisher)

영상 파일을 열고 30ms 간격으로 /video 토픽에 프레임 퍼블리시

영상이 끝나면 rclcpp::shutdown() 호출

cap_.open(video_path);
pub_ = this->create_publisher<sensor_msgs::msg::Image>("video", 10);
timer_ = this->create_wall_timer(30ms, std::bind(&VideoPublisher::timer_callback, this));

cv::Mat frame;
cap_.read(frame);
auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
pub_->publish(*msg);


OpenCV로 프레임을 읽고, cv_bridge를 통해 ROS 메시지로 변환 후 퍼블리시

5.2 구독자 노드 (LineDetector)
subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/video", rclcpp::QoS(10),
    std::bind(&LineDetector::image_callback, this, std::placeholders::_1));


/video 토픽으로 영상 구독

image_callback 함수에서 영상 처리 수행

5.3 ROI 설정 및 밝기 보정
int roi_height = 90;
cv::Rect roi_rect(0, frame.rows - roi_height, frame.cols, roi_height);
cv::Mat roi = frame(roi_rect).clone();
double shift = 140.0 - cv::mean(roi)[0];
roi.convertTo(roi, -1, 0.7, shift);


하단 90픽셀 영역만 라인 검출 대상으로 사용

평균 밝기를 기준으로 밝기 보정

설명: ROI 기반 처리로 영상 전체를 처리하지 않아 성능 향상

5.4 이진화 및 윤곽 검출
cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);
cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


그레이스케일 변환 후 임계값 이진화

findContours로 라인 후보 영역 검출

설명: threshold 값 140은 환경에 따라 변경 가능, 향후 adaptive threshold 적용 가능

5.5 라인 중심 추적

이전 중심 기준으로 가장 가까운 후보 선택

max_speed와 lost_count_를 활용하여 부드러운 중심 이동

double error = frame.cols / 2.0 - prev_center_x_;
RCLCPP_INFO(this->get_logger(), "error:%d, time:%.4f sec", (int)error, elapsed_time);


화면 중심과 라인 중심의 오차 계산

영상 처리 소요 시간 측정 및 출력

설명:

max_speed: 중심 이동 속도 제한

lost_count_: 라인을 놓쳤을 때 보정 횟수

6. 실험 결과

라인 검출과 중심점 표시 정상 수행

처리 속도: 평균 33ms/프레임



7. 분석 및 고찰

ROI 기반 처리로 불필요한 영역 제거 → 성능 향상

밝기 변화가 큰 환경에서는 adaptive thresholding 필요

라인을 놓쳤을 때 lost_count_와 max_speed를 이용한 부드러운 복귀 가능

중심점 smoothing으로 불안정한 움직임 최소화



8. 결론

ROS2와 OpenCV를 활용한 라인 검출 및 중심 오차 계산 실습 성공

실시간 영상 처리 및 라인트레이싱 알고리즘 이해에 도움
