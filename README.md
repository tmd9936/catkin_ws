# catkin_ws
# ROS 공부용
# beginner_tutorial 실행 
- $ roscore
- $ rosrun beginner_tutorial topic_publisher
- $ rosrun beginner_tutorial topic_subscriber

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

