# mobile_manip

 Mobile manipulator Task Planning implementaion  by using legacy method (BasePlacement)

## Experiment 
'ros-noetic', 'ubuntu 20.04'

#### - Simulation Scence (Coppeliasim)
![SimEnv](https://user-images.githubusercontent.com/49723556/151513552-4a18bd52-326e-4349-b107-844899f97b59.png)

#### - IRM structure about sampled Grasp Pose from Point cloud (Red is High value)
![image](https://user-images.githubusercontent.com/49723556/151492932-8656a287-cd8e-49da-adbd-e60db22570e8.png)

#### - RL sequence visulazation using smach
![image](https://user-images.githubusercontent.com/49723556/152644945-2308bda1-6664-4fd2-9692-59a77cfc3e94.png)


#### Excute demo program

1. excute 'roscore' for CoppeliaSim rosInterface.
2. turn on 'Panda_Husky_test.ttt' for test. 
3. run 'roslaunch  TAMP_interface interface.launch' on your terminor.
4. run 'rosrun  smach_viewer smach_viewer.py' you can see whole process on smach viewer!! 

# topic name

 * PointCloud2 : '/k4a/depth_registered/points'  
 * MakerArray : 'ws_IRM' 

## TODO

1. make IRM with numerical method is inefficient --> get Distribuiton using GAN (fast sampling)
2. How to use Quality Measure?? 
3. study RL / ML for efficient implementation.
4. apply NVIDA Issac Gym

## Reference
- Vahrenkamp, N., Asfour, T., Metta, G., Sandini, G., & Dillmann, R. (2012). Manipulability analysis. 2012 12th IEEE-RAS International Conference on Humanoid Robots (Humanoids 2012)

- https://github.com/jkw0701/robocare_motion_planner.git
- https://gitlab.com/Simox/simox.git

## ..
if you find good reference fir project ... pleaze share me... sammin9614@gmail.com
