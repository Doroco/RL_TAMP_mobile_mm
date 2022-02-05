# arm_motion_planner

## description

Rapidly-exploring Random Tree(RRT) 기반 모션 계획 알고리즘 

## Auto-install

> 이 스크립트는 `ros-melodic` 이 설치되어 있고, `~/social-root` 폴더가 있다고 가정합니다.
> 또한, `~/social-root/catkin_ws/devel/include/` 에 `Traj. Smoothing Lib` 에 대한 symbolic link를 생성합니다.

**via curl**

``` sh
bash -c "$(curl -fsSL https://raw.githubusercontent.com/ryul1206/setting-my-env/master/install-snu.sh)"
```

## Manual-install

> 아래 서술된 설치방법에는 맞지 않는 내용이 있습니다.
> 따라서 오류가 수정된 auto-install을 권장합니다.
> 메뉴얼 설치 가이드는 패키지 정리가 거의 마무리되면 업데이트 하겠습니다.

### 1. ROS

ros-kinetic 버전에서 테스트 되었습니다.

다음 링크에서 자세한 설치 방법을 확인할 수 있습니다. 
[ROS Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)

#### 추가 설치 패키지

TRAC-IK Kinematics Solver

``` bash
sudo apt-get install ros-kinetic-trac-ik
```

kdl_parser

``` 
sudo apt-get install ros-kinetic-kdl-parser
```

### 2. RBDL

Urdf Reader을 사용하기 위해 최신버전 설치를 요함.

``` 
wget https://bitbucket.org/rbdl/rbdl/get/849d2aee8f4c.zip
unzip 849d2aee8f4c.zip
cd rbdl-rbdl-849d2aee8f4c
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```

### 3. Boost

### 4. Eigen3

### 5. Trajectory Smoothing Library

``` bash
git clone https://github.com/ggory15/trajectory_smoothing --recursive
cd trajectory_smoothing
mkdir build && cd build
cmake ..
make
sudo make install
```

## Service

~plan_left_arm_motion

~plan_right_arm_motion

### Request 

~current_joint_state(sensor_msgs:: JointState)

    Manipulator's initial joint position 

~target_ee_pose(geometry_msgs:: Pose)

    End-effector's target pose defined in the robot frame, not in the global frame

~Pose_bound(arm_motion_planner:: PoseConstraint)

    

~Obstacle3D(arm_motion_planner:: Obstacle3D)

~current_mobile_state(geometry_msgs:: Pose2D)

~plan_only(std_msgs:: Bool)

### Response

(trajectory_msgs:: JointTrajectory)

## How to use

1. V-REP 실행 (vrep remote api로 시뮬레이션이 진행되므로, 실제 로봇 연결 상황이 아니면 V-REP이 실행되어 있어야 함)

2. 시뮬레이션 환경 및 모션 플래너 실행

로봇 제어기 실행

``` 
rosrun mobile_manipulator_controller main
```

모션 플래너 server 실행

``` 
roslaunch arm_motion_planner left_arm_server.launch
```

모션 플래너 client 실행
```
roslaunch arm_motion_planner left_arm_client.launch
