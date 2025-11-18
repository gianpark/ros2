실습과제1

Jetson nano, wsl2-ubuntu, windows의 ip주소를 출력하라.

Jetson nano, wsl2-ubuntu에 각각 ifconfig 명령어 입력하여 ip주소 출력, windows ipconfig명령어 입력하여 ip주소 출력
 
같은 네트워크에 있는지 설명하라.

같은 공유기 연결시 파워셸,우분투 ip주소 동일, jetson nano의 ip주소 앞3자리가 같음.

Jetson nano 에서 환경변수 ROS_DOMAIN_ID값을 확인하라.

Jetson nano의 홈디렉터리 아래에 있는 ~/.bashrc에서

ROS_DOMAIN_ID= 1, ROS_NAMESPACE=jetson1 로 변경

$ cd -> 홈디렉토리로 이동

$ source .bashrc -> 수정된 .bashrc 파일 실행

$ echo $ROS_DOMAIN_ID -> 1 확인

$ echo $ROS_NAMESPACE jetson1 확인


wsl2-ubuntu 에서 환경변수 ROS_DOMAIN_ID값을 확인하라.


Wsl2-ubuntu20.04의 스크립트파일 ~/.bashrc 파일에서 환경변수

ROS_DOMAIN_ID= 1, ROS_NAMESPACE=jetson1 로 수정

$ cd -> 홈디렉토리로 이동

$ source .bashrc -> 수정된 .bashrc 파일 실행

$ echo $ROS_DOMAIN_ID -> 1 확인

$ echo $ROS_NAMESPACE jetson1 확인


2개가 로봇번호와 같은지 확인하라
