결과 동영상 : https://youtu.be/ZymWpI6bNIE

ROS2 기반 영상 퍼블리싱 및 개선형 라인 추적 실습 보고서
1. 작성자

이름: 박기안

학번: 2001186

작성일: 2025/12/09

2. 실습 목적

ROS2와 OpenCV를 활용하여 동영상 데이터를 ROS2 토픽으로 퍼블리시하고, 구독하여 실시간 라인 추적 수행

기존 단순 라인 검출에서 후보 필터링, 속도 제한, 추적 손실 처리를 추가하여 안정적인 라인 추적 알고리즘 구현

실시간 처리 성능 측정 및 영상 디버깅 이해

3. 실습 환경

OS: Ubuntu 22.04 LTS

ROS2 버전: Humble

OpenCV: 4.x

개발 언어: C++17

개발 도구: Visual Studio Code / Colcon 빌드

4. 소스코드 구조 및 설명
4.1 VideoPublisher 노드

노드 이름: video_publisher

기능: 지정된 동영상(7_lt_ccw_100rpm_in.mp4)을 읽어 "video" 토픽으로 퍼블리시

퍼블리시 주기: 30ms (약 33Hz)

영상 처리 방식:

OpenCV VideoCapture로 동영상 읽기

cv_bridge::CvImage를 이용해 ROS Image 메시지로 변환 후 퍼블리시

종료 조건: 영상 끝 도달 시 ROS2 종료

auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
pub_->publish(*msg);

4.2 LineDetector 노드

노드 이름: line_detector

기능: "video" 토픽 구독 후 라인 검출 및 추적

특징 및 개선점:

ROI 기반 라인 검출: 영상 하단 90픽셀, 폭 640픽셀 영역만 처리

밝기 보정: ROI 평균 밝기를 목표값 140으로 조정, 대비 및 가중치 적용

이진화 및 윤곽 추출: cv::threshold + cv::findContours로 라인 후보 검출

후보 필터링:

최소 면적: 60

최소 폭/높이: 3 / 5

세로/가로 비율 >= 0.10

이전 중심값 기반 후보 선택:

이전 프레임 중심값과 가장 가까운 후보 선택

최대 이동 거리 제한(max_jump) 및 속도 제한(max_speed, reappear_speed) 적용

lost_count 카운터로 라인 상실 시 보정

에러 계산: 영상 중앙 대비 라인 중심 오차 계산

시각화:

Original Frame: 입력 영상

Binary with Overlay: 후보 영역과 선택 라인 표시

double error = frame_w / 2.0 - prev_center_x_;
RCLCPP_INFO(this->get_logger(), "error:%d, time:%.4f sec", (int)error, sec);

4.3 개선된 추적 로직

lost_count: 라인을 감지하지 못하면 증가, 일정 카운트 이상 시 재출현 처리

속도 제한: 라인 이동 속도 제한을 적용하여 급격한 점프 방지

재출현 처리: 라인 소실 후 재출현 시 부드러운 추적 적용

5. 실습 과정

ROS2 워크스페이스 빌드:

colcon build --symlink-install
source install/setup.bash


퍼블리셔 노드 실행:

ros2 run <package_name> video_publisher


라인 추적 노드 실행:

ros2 run <package_name> line_detector


OpenCV 창을 통해 영상 확인 및 라인 추적 상태 관찰

로그를 통해 에러 값 및 처리 시간 확인

6. 결과

영상 퍼블리셔가 정상적으로 동영상 스트리밍

라인 추적 노드가 ROI 영역에서 후보 필터링 후 안정적으로 라인 중심 추적

라인 소실 상황에서 lost_count와 속도 제한 적용으로 안정적 복원

실시간 처리 속도: 프레임당 약 0.02~0.03초

예시 로그

[INFO] [line_detector]: error:-10, time:0.0287 sec
[INFO] [line_detector]: error:5, time:0.0251 sec


OpenCV 시각화

원본 영상: Original Frame

라인 검출 + 후보 표시: Binary with Overlay (빨간: 선택, 파랑: 후보)

7. 결론

ROS2 기반 영상 퍼블리싱과 라인 추적 기능 정상 동작 확인

lost_count 및 속도 제한 적용으로 라인 추적 안정성 향상
