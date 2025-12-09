결과동영상 : https://youtu.be/g3LLreHdY8A

ROS2 기반 영상 퍼블리싱 및 라인 검출 실습 보고서
1. 작성자

이름: 박기안

학번: 2001186

작성일: 2025/12/09

2. 실습 목적

ROS2와 OpenCV를 이용하여 동영상 데이터를 ROS 토픽으로 퍼블리시하고 구독하는 방법 이해

구독한 영상을 ROI 기반으로 전처리 및 라인 검출을 수행하고, 에러 값 계산 및 시각화

실시간 영상 처리 속도 측정과 디버깅을 위한 로깅 이해

3. 실습 환경

OS: Ubuntu 22.04 LTS

ROS2 버전: Humble

OpenCV: 4.x

사용 언어: C++17

개발 도구: Visual Studio Code / Colcon 빌드

4. 소스코드 설명
4.1 VideoPublisher 노드

노드 이름: video_publisher

역할: 지정한 동영상 파일(/home/linux/ros2_ws/simulation/5_lt_cw_100rpm_out.mp4)을 읽고, ROS2 토픽 "video"로 퍼블리시

주기: 30ms(약 33Hz)

주요 기능:

cv::VideoCapture를 이용해 영상 파일 읽기

cv_bridge::CvImage를 통해 OpenCV 이미지 → ROS2 Image 메시지 변환

퍼블리시 실패 시 로그 출력 및 종료 처리

auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
pub_->publish(*msg);

4.2 LineDetector 노드

노드 이름: line_detector

역할: "video" 토픽에서 영상을 구독하여 라인 검출 수행

주요 기능:

ROI 설정: 영상 하단 90픽셀 영역을 라인 검출 대상으로 지정

밝기 보정: ROI 평균 밝기를 목표값(128)으로 조정

이진화: cv::threshold를 통해 라인 검출용 바이너리 영상 생성

연결 요소 분석: cv::connectedComponentsWithStats를 사용하여 후보 영역 추출

라벨링 및 후보 선정: 이전 프레임 중심값 기준 가장 가까운 후보 선택

에러 계산: 영상 중앙 대비 라인 중심 오차 계산

시각화:

원본 영상: Original Frame

라인 검출 결과: Binary with Overlay

double error = frame.cols/2.0 - cx_final;
RCLCPP_INFO(this->get_logger(), "error:%d, time: %.4f sec",
            (int)std::round(error), t);


선택 후보는 빨간색 사각형과 중심점으로 표시, 나머지는 파란색 사각형

5. 실습 과정

ROS2 워크스페이스 빌드:

colcon build --symlink-install
source install/setup.bash


퍼블리셔 노드 실행:

ros2 run <package_name> video_publisher


라인 검출 노드 실행:

ros2 run <package_name> line_detector


OpenCV 창을 통해 원본 영상 및 라인 검출 결과 확인

로그를 통해 에러 값 및 처리 시간 확인

6. 결과

영상 퍼블리셔가 정상적으로 동영상을 토픽에 퍼블리시함

라인 검출 노드는 ROI 영역에서 라인을 성공적으로 검출

라인 중심점이 영상 중앙에서 얼마나 벗어나는지 error 값으로 확인 가능

실시간 처리 속도는 프레임 당 약 0.01~0.03초로 측정됨

예시 로그:

[INFO] [line_detector]: error:12, time: 0.0234 sec

[INFO] [line_detector]: error:-5, time: 0.0217 sec


OpenCV 시각화

Original Frame: 입력 영상 그대로 표시

Binary with Overlay: 라인 후보와 선택 라인 표시, 중심점 표시

7. 결론

ROS2 토픽을 통해 동영상 스트리밍이 가능함을 확인

ROI와 밝기 보정을 통해 노이즈가 많은 영상에서도 안정적인 라인 검출 가능

이전 프레임 중심값을 활용한 후보 선택으로 라인 추적 정확도가 향상
