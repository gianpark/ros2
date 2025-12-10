5_lt_cw_100rpm_out 실행결과 : https://youtu.be/2kQ4_Qzcz9k

6_lt_ccw_100rpm_out 실행결과 : https://youtu.be/18Mz9PqtEyM

7_lt_ccw_100rpm_in 실행결과 : https://youtu.be/9GUr5OLuwbI

8_lt_cw_100rpm_in 실행결과 : https://youtu.be/fUiWoAYmaro

ROS2 기반 라인 검출(Line Detection) 실습 결과 보고서

작성자: 박기안

학번: 2001186

작성일: 2025.12.10


1. 실습 개요

본 실습은 ROS2(Rolling) 환경에서 OpenCV를 사용하여 영상 기반 라인 검출(Line Detection) 기능을 구현하고,

ROS2의 퍼블리셔·서브스크라이버 구조를 통해 실시간 원격접속과 라인 중심 추적 알고리즘을 실행하는 것을 목표로 한다.

본 프로젝트는 다음 두 개의 노드로 구성된다.

VideoPublisher

mp4 파일을 읽어 영상 프레임을 /video1 토픽으로 퍼블리시

line_subscriber

/video1 토픽의 영상을 구독

ROI(Region of Interest) 추출

밝기 보정 → 이진화 → 윤곽 검출

라인 중심 추정 및 사라짐 복구 알고리즘 적용

라인 중심의 에러(error) 계산

2. 시스템 구성도
3. 
[Video File] → [VideoPublisher] → /video1 → [LineDetector] → Error 출력 및 시각화


VideoPublisher는 30ms(≈33FPS) 주기로 프레임을 publish 하며, line_subscriberr는 이 영상을 받아 실시간 처리를 수행한다.

3. 구현 상세
   
   
3.1 VideoPublisher 클래스

VideoPublisher 클래스는 다음 기능을 수행한다.

cv::VideoCapture로 mp4 파일 오픈

OpenCV(Mat)를 ROS2 sensor_msgs::Image로 변환(cv_bridge 사용)

30ms 주기로 publish

주요 코드 특징

C++17의 std::bind와 클래스 멤버 함수 바인딩

스마트 포인터(std::shared_ptr) 기반 ROS2 노드 구조

실패 처리: 영상이 끝나면 rclcpp::shutdown() 수행

3.2 line_subscriber 클래스

라인 검출 알고리즘은 다음 단계를 따른다.

3.2.1 ROI 설정

전체 프레임 중 하단 90px 구간만 활용한다.

int roi_height = 90;

int roi_y = frame_h - roi_height;


하단을 사용하는 이유:

라인이 촬영 화면의 아래쪽에 존재하는 경우가 많음

불필요한 연산 감소 → 처리 속도 증가

3.2.2 밝기 보정

영상의 주변 환경 변화(밝기/조도)에 따라 threshold 안정화를 위해 밝기 보정 수행.

double shift = target_mean - cv::mean(roi)[0];

roi.convertTo(roi, -1, 0.7, shift);


목표 밝기: 평균 140 수준

gain=0.7, bias=shift 적용

3.2.3 이진화 & 윤곽 검출

cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


윤곽(contour)의 bounding box를 분석하여 라인 후보를 걸러냄.

필터 조건

영역 60px 이상

최소 너비/높이 조건

세로 방향 비율(rect.height / rect.width ≥ 0.1)

3.2.4 ROI 상단(Top 30%) 제거

급커브 상황에서 라인이 상단으로 빠르게 벗어나는 경우 오검출 방지:

if (rect.y + rect.height < roi_height * 0.30)

    continue;

3.2.5 라인 중심 추정 및 보정 알고리즘

① 정상 검출 시

가장 이전 중심(prev_center_x)과 가까운 contour 선택

이동 속도 제한

정상 상태: max_speed = 12

재등장 상태: reappear_speed = 6

1차 지연 필터 (exponential smoothing)

prev_center_x_ = prev_center_x_ * 0.7 + target * 0.3;

② 라인이 사라진 경우(lost)

사라진 방향을 추정하여 중심을 보정한다.

조건	해석	보정 위치

prev_center_x < 0.3W	왼쪽 상단으로 사라짐	화면 3/5 지점

prev_center_x > 0.7W	오른쪽 상단으로 사라짐	화면 1/5 지점

중앙	급커브	이전 위치 유지

이 추정은 C++의 조건 분기를 통하여 STL 기반 논리 처리로 구현된다.

3.2.6 error 계산

double error = frame_w / 2.0 - prev_center_x_;


양수: 라인이 오른쪽으로 치우침

음수: 왼쪽으로 치우침

3.2.7 실행 시간 측정

auto start = std::chrono::steady_clock::now();
...
auto end = std::chrono::steady_clock::now();

std::chrono::duration<double> elapsed = end - start;


약 0.02 ~ 0.04초(25 ~ 45FPS)의 처리 속도를 보임.

4. 실행 결과
   
4.1 출력 화면

프로그램은 두 개의 창을 띄움:

Original Frame – 원본 영상

Binary with Overlay – 이진화 + contour 표시 + 라인 중심 표시

라인이 검출되는 경우 빨간 원으로 중심 표시.

4.2 로그 출력 예시

error:-38, time:0.0285 sec

error:-42, time:0.0321 sec

error:15,  time:0.0300 sec


error는 라인 중심의 프레임 중앙 대비 편차

time은 처리 시간(sec)

5. 고찰
   
5.1 성능 분석

ROI 기반 연산 감소로 평균 30~40FPS 처리 가능

contour 기반 라인 검출은 속도가 빠르지만 급커브 대응이 어려움

"사라짐 복구 알고리즘" 적용으로 급격한 상황에서도 안정적인 중심 추정 가능

5.2 한계점

단순 threshold 기반 이진화는 조명 변화에 취약

contour 기반은 라인 폭이 넓거나 여러 요소가 겹칠 때 오검출 가능





6. 결론

이번 실습에서는 ROS2와 OpenCV를 활용하여 다음 기능을 성공적으로 구현하였다.

mp4 영상을 ROS2 토픽으로 송출 (VideoPublisher)

영상 기반 ROI 추출 및 라인 후보 검출(LineDetector)

라인 중심 추정, 소실 복구 로직 포함

실시간 성능 모니터링(처리시간, error 출력)

안정적인 30FPS 이상의 라인 검출 시스템 완성
