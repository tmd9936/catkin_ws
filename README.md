# catkin_ws
# ROS 공부용
[ROS공식 사이트]http://wiki.ros.org/
# beginner_tutorial 실행 
- $ roscore
- $ rosrun beginner_tutorial topic_publisher
- $ rosrun beginner_tutorial topic_subscriber


# 설치
```
우분투 설치 iso파일 ( 18.04 버전)
https://releases.ubuntu.com/18.04.5/?_ga=2.139951610.815617854.1607320047-1046809282.1607320047

외장하드관리 툴(rufus)를 이용해서 외장하드에 iso파일 설치
http://rufus.ie/
 diskmgmt.msc실행
 c드라이브에서 20GB정도 볼륨축소
 refus시작, 부트는 다운받은 ubuntu iso파일, usb를 꽂고 usb선택 후 바로 시작버튼 클릭
BIOS창 열기 (컴퓨터 로딩중 F2)
 Boot메뉴
 부팅순서를 ubunt가 위로가게 설정
 부팅USB로 부팅 
----
우분투 설치화면
 third파티 체크
 somthig else
 free space
 mount point /
 마운트 포인트에 /가 있는지 확인
----

설치 과정
 리눅스 듀얼 부팅으로 설치
 LInux 18.04 설치
 부팅 USB만들기 
 메모리 할당
 BIOS 설정
 설치
```

## ~/.bashrc에 아래 설정 추가

```bash
# 터미널을 열때마다 실행
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
```

## 패키지가 없다고 뜨는 경우
- $ source ~/catkin_ws/devel/setup.bash

# 명령어
마스터 실행
- roscore

노드 실행
- rosrun [패키지명] [노드명]

그래프 확인
- rqt_graph
- rosrun rqt_graph rqt_graph

Publisher와 Subscriber의 토픽들 확인
- rostopic list -v



# 번외 (turtlesim)
GUI화면 노드
- rosrun turtlesim turtlesim_node

키 받는 노드
- rosrun turtlesim turtle_teleop_key




