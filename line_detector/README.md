결과 동영상 : https://youtu.be/ZymWpI6bNIE

1. 제목

ROS2 기반 라인트레이싱 영상 처리 실습 보고서

2. 작성자

이름: 박기안

학번: 2001186

작성일: 2025/12/09

3. 목적

ROS2와 OpenCV를 활용하여 동영상에서 라인을 검출하고, 라인 중심과 화면 중심의 오차를 계산한다.


자율주행 로봇 라인트레이싱 알고리즘 개발에 적용 가능한 기반을 마련한다.

4. 실험 환경

하드웨어: PC (CPU: AMD Ryzen 5 5500U, RAM: 16GB, GPU: 없음)

소프트웨어: Ubuntu 22.04, ROS2 Humble, OpenCV 4.8.1, C++17

사용 패키지: rclcpp, sensor_msgs, cv_bridge


5. 코드 구조 및 처리 과정
5.1 Class 구조

VideoPublisher 클래스

rclcpp::Node 상속

영상 파일을 열고, cv::VideoCapture를 이용해 프레임을 읽어 /video 토픽에 퍼블리시

STL std::bind를 활용해 타이머 콜백 연결

LineDetector 클래스

rclcpp::Node 상속

/video 토픽 구독

image_callback에서 라인 검출 수행

STL std::vector 사용 → 윤곽선(contours) 저장 및 처리

설명: Class와 STL을 활용하여 코드 모듈화 및 동적 데이터 관리 용이

5.2 STL 활용 예시

윤곽선 저장

std::vector<std::vector<cv::Point>> contours;
cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


최적 라인 후보 선택

double min_dist = 1e9;
for (const auto &cnt : contours) {
    cv::Rect rect = cv::boundingRect(cnt);
    double candidate_x = rect.x + rect.width / 2;
    double dist = std::abs(candidate_x - prev_center_x_);
    if (dist < min_dist) { min_dist = dist; best_center_x = candidate_x; }
}


STL std::vector를 사용하면 윤곽선 개수에 상관없이 동적으로 처리 가능

5.3 ROI 설정 및 밝기 보정
int roi_height = 90;
cv::Rect roi_rect(0, frame.rows - roi_height, frame.cols, roi_height);
cv::Mat roi = frame(roi_rect).clone();
double shift = 140.0 - cv::mean(roi)[0];
roi.convertTo(roi, -1, 0.7, shift);


ROI 기반 처리로 영상 전체를 처리하지 않아 성능 향상

5.4 이진화 및 중심 추적
cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);


중심점 smoothing

prev_center_x_ = prev_center_x_ * 0.7 + target * 0.3;


STL과 class 멤버 변수를 활용하여 이전 중심 저장 및 계산

5.5 오차 계산
double error = frame.cols / 2.0 - prev_center_x_;
RCLCPP_INFO(this->get_logger(), "error:%d, time:%.4f sec", (int)error, elapsed_time);


화면 중심과 라인 중심의 오차 출력

6. 실험 결과

라인 검출과 중심점 표시 정상 수행

처리 속도: 평균 33ms/프레임


7. 분석 및 고찰

STL std::vector를 이용한 윤곽선 저장 → 메모리 관리 효율적

Class 구조로 코드 모듈화 → 유지보수 및 확장 용이

ROI 기반 처리로 불필요한 영역 제거 → 처리 속도 향상

밝기 변화가 큰 환경에서는 adaptive thresholding 필요

중심점 smoothing으로 불안정한 움직임 최소화


8. 결론

ROS2와 OpenCV, STL과 class 기반 설계를 활용한 라인 검출 실습 성공

실시간 영상 처리 및 라인트레이싱 알고리즘 이해에 도움

