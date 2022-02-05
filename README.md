# RL_mobile_manip

### I'm still Testing...  :(          
#### Please wait for month...!
Reinfore Learning / Mobile manipulator Task Planning implementaion  by using DDPG

- Simulation Scence (Coppeliasim)
![SimEnv](https://user-images.githubusercontent.com/49723556/151513552-4a18bd52-326e-4349-b107-844899f97b59.png)

- IRM structure about sampled Grasp Pose from Point cloud (Red is High value)
![image](https://user-images.githubusercontent.com/49723556/151492932-8656a287-cd8e-49da-adbd-e60db22570e8.png)

## TODO

1. Lazy Collision Check (IRM sturcture)
- setObstacleRegion() 2D Collision Bound 설정하기
2. Contrained Motion Planning
- Door Openning을 실행하기 위해서 Task Constraint 지정 (Task Frame 설정하기)
- Task Constrined RRT를 시행하기 위해서 기존 Arm motion planner에 First-Order Retraction을 추가로 구현하기 
