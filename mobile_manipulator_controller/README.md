# mobile_manipulator_controller
[SRP_main]: https://gitlab.com/social-robot/socialrobot_motion_planner

- Version 1.0.0
- [[Go to the Motion Planner Module in Social Robot Project]][SRP_main]

## description
- 모바일 매니퓰레이터의 제어 모듈

## installation
### 1.ROS
ros-kinetic 버전에서 테스트 되었습니다.

다음 링크에서 자세한 설치 방법을 확인할 수 있습니다. 
[ROS Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### 2.Boost

### 3.Eigen3

### 4.V-REP
socialrobot 프로젝트는 v-rep 시뮬레이터를 사용합니다.(version 3.5.0 이상)

다운로드: [V-REP Linux](http://www.coppeliarobotics.com/downloads.html)

압축 해제
```
tar -xvzf {file_name}
```
실행
```
cd {v-rep directory path}./vrep.sh
```
ROS와 연동하기 위해 socialrobot 프로젝트는 remote api를 사용합니다. 활성화하기 위해 vrep 디렉토리 안의 *remoteApiConnections.txt* 파일을 수정합니다.
```
portIndex1_port             = 19997
```


## Input & Ouput
Input 
- 로봇 팔의 목표 관절 위치 경로
- 모바일 로봇의 목표 자세 경로

Output
- 로봇 팔의 목표 관절 위치 / 속도 / 토크
- 모바일 로봇의 선속도 / 각속도


## how to use
제어기 실행
```
rosrun mobile_manipulator_controller main
