# mobile_motion_planner
[SRP_main]: https://gitlab.com/social-robot/socialrobot_motion_planner

- Version 1.0.0
- [[Go to the Motion Planner Module in Social Robot Project]][SRP_main]

## description
-Non-holonomic 제한 조건을 고려한 Rapidly-exploring Random Tree(RRT) 기반 모바일 로봇 모션 계획 알고리즘 

## installation
### 1.ROS
ros-kinetic 버전에서 테스트 되었습니다.

다음 링크에서 자세한 설치 방법을 확인할 수 있습니다. 
[ROS Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### 2.Boost

### 3.Eigen3

### 4.Trajectory Smoothing Library
```bash
git clone https://github.com/ggory15/trajectory_smoothing --recursive
cd trajectory_smoothing
mkdir build && cd build
cmake ..
make
sudo make install
```

### 5.V-REP
socialrobot 프로젝트는 v-rep 시뮬레이터를 사용합니다.(version 3.5.0 이상)

## Messages
- MobileTrajectory.msg
  - points([geometry_msgs/Pose2D])
    - 2D 경로의 점들

- Obstacle2D.msg
  - x ([std_msgs/Float64])
    - 장애물의 x 좌표
  - y ([std_msgs/Float64])
    - 장애물의 y 좌표
  - radius ([std_msgs/Float64])
    - 장애물의 반지름

## Parameters
### YAML File(mobile_motion_planner/config)
- position_limit_lower / position_limit_upper
  - 모바일의 위치 반경 (x, y, theta)

ROS와 연동하기 위해 socialrobot 프로젝트는 remote api를 사용합니다. 활성화하기 위해 vrep 디렉토리 안의 *remoteApiConnections.txt* 파일을 수정합니다.
```
portIndex1_port             = 19997
```
## Service
### plan_mobile_motion(mobile_motion_planner/srv/plan_mobile_motion.srv)

Request 
- current_mobile_state([geometry_msgs/Pose2D](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose2D.html))
  - 전역 좌표계 기준 현재 모바일 로봇의 위치(x,y,theta)

- target_mobile_pose([geometry_msgs/Pose2D](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose2D.html))
  - 전역 좌표계 기준 목표 모바일 로봇의 위치

- Obstacle2D([mobile_motion_planner/Obstacle2D](mobile_motion_planner/msg/Obstacle2D.msg))
  - 원으로 표시된 2차원 상의 장애물 정보

Response
- mobile_trajectory([mobile_motion_planner/MobileTrajectory]
  - 목표 모바일 위치에 도달하는 경로

## Messages
- MobileTrajectory.msg
  - points([geometry_msgs/Pose2D])
    - 2D 경로의 점들

- Obstacle2D.msg
  - x ([std_msgs/Float64])
    - 장애물의 x 좌표
  - y ([std_msgs/Float64])
    - 장애물의 y 좌표
  - radius ([std_msgs/Float64])
    - 장애물의 반지름

## Parameters
### YAML File(mobile_motion_planner/config)
- position_limit_lower / position_limit_upper
  - 모바일의 위치 반경 (x, y, theta)

- mobile_length / mobile_width
  - 모바일 로봇의 길이와 폭

## how to use
모션 플래너 server 실행
```
roslaunch mobile_motion_planner server.launch
```
모션 플래너 client 실행
```
roslaunch mobile_motion_planner client.launch
