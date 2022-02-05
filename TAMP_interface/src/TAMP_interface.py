#!/usr/bin/env python
from operator import imod
import numpy
import rospy

import threading

import smach
import numpy
from smach import StateMachine, Concurrence
import smach_ros
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import task_assembly.srv
from task_assembly.srv import plan_mobile_motion, plan_mobile_motionRequest, plan_arm_motionResponse
from task_assembly.srv import plan_arm_motion, plan_arm_motionRequest, plan_mobile_motionResponse
from task_assembly.srv import base_placement, base_placementRequest, base_placementResponse
from task_assembly.srv import sim_request, sim_requestRequest, sim_requestResponse

from task_assembly.msg import GraspConfigList
from task_assembly.msg import GraspConfig
from task_assembly.msg import Obstacle2D
from task_assembly.msg import MobileTrajectory
from geometry_msgs.msg import Pose2D
from math import pi
from enum import Enum

# 외부클래스 호출
class RunCommand(Enum):
    SAMPLER = 0
    ARM_FEXIBILITY_CHECK = 1
    MOBILE_FEXIBILITY_CHECK = 2
    PLANNING_FAILED = 3
    SIM_STATE_CHECK = 4

class SimRunState(Enum):
    RUN_SIMULATION = 0
    STOP_SIMULATION = 1
    QUIT_SIMULATION = 2
    RESTART_SIMULATION = 3

#  asumtion
#  1. 장애물의 위치를 모두 안다.
#  2. Smooth path가 언제나 존재한다.
#  3. perceoption 알고리즘과 Affordacne를 통해 유추한 Grasp Pose가 주어져있고, 그 Pose들이 정확하다.

# 일을 수행하기위해서 Grasp Pose를 찾아보아요
class getRandomWork(smach.State) :
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['succeeded','failed'],
                                input_keys=['GraspConfig'])

        # 우선하나의 Grasp만 정의를 해볼께요
        self.doorHandle = GraspConfigList()
        self.doorHandle.header.frame_id = "rgb_optical_frame"

        ####### define known grasp pose ######################
        tmpGrasp = GraspConfig()
        tmpGrasp.position.x = 0.005349214755890379
        tmpGrasp.position.y = 0.29167141119290857
        tmpGrasp.position.z = 3.045161183218673
        tmpGrasp.approach.x = 0.0016649097536617556
        tmpGrasp.approach.y = 0.0023747117365735242
        tmpGrasp.approach.z = 0.9999957944009967
        tmpGrasp.binormal.x = 0.9999985956127705
        tmpGrasp.binormal.y = -0.00019591173747848635
        tmpGrasp.binormal.z = -0.0016644491815872556
        tmpGrasp.axis.x = 0.00019195832654583298
        tmpGrasp.axis.y = 0.9999971611773502
        tmpGrasp.axis.z = -0.002375034576921272
        tmpGrasp.width.data = 0.012007035315036774
        tmpGrasp.score.data = -182.82186889648438
        tmpGrasp.sample.x = -0.001284144353121519
        tmpGrasp.sample.y = 0.2917202115058899
        tmpGrasp.sample.z = 3.0651721954345703
        self.doorHandle.grasps.append(tmpGrasp)

    def execute(self, userdata) :
        print(len(userdata.simRunState.points))
        return "succeeded"

# 시뮬레이션 실행을 정의
class SimInterface(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['succeeded'],
                                input_keys=['simRunState'])
    def execute(self, userdata) :
        print(len(userdata.simRunState.points))
        return "succeeded"

    # def Search_and_Sampler(userdata,response): 
    #     # print(len(response.mobile_trajectory.points))
    #     userdata.mobile_plan_res.points = response.mobile_trajectory.points
    #     if len(userdata.mobile_plan_res.points) : 
    #         return "succeeded"
    #     else :
    #         return "aborted"

def mobile_fexibility_check(userdata,response): 
    # print(len(response.mobile_trajectory.points))
    userdata.mobile_plan_res.points = response.mobile_trajectory.points
    if len(userdata.mobile_plan_res.points) : 
        return "succeeded"
    else :
        return "replanning"

def arm_fexibility_check(userdata,response): 
    # print(len(response.mobile_trajectory.points))
    userdata.mobile_plan_res.points = response.mobile_trajectory.points
    if len(userdata.mobile_plan_res.points) : 
        return "succeeded"
    else :
        return "replanning"

# main
def main():
    rospy.init_node('TAMP_Scence')

    # Create the top level SMACH state machine
    sm = StateMachine(outcomes=['succeeded','aborted','preempted','failed'])
    
    ########################################### Define User-Data ######################################################################
    sm.userdata.simRunState = SimRunState.RUN_SIMULATION.value
    sm.userdata.armTraj = plan_arm_motionResponse()
    sm.userdata.mobileTraj = MobileTrajectory()
    sm.userdata.graspConfig = GraspConfig()

    mmreq = plan_mobile_motionRequest()
    mmreq.current_mobile_state.x = 0
    mmreq.current_mobile_state.y = 0
    mmreq.current_mobile_state.theta = 0

    mmreq.target_mobile_pose.x = 4.9
    mmreq.target_mobile_pose.y = -2.3
    mmreq.target_mobile_pose.theta = -pi/2.0

    obs1 = Obstacle2D()
    obs1.x.data = 2.7251
    obs1.y.data = -0.5276
    obs1.radius.data = 0.6

    mmreq.Obstacles2D.append(obs1)

    coverdata = Pose2D()
    coverdata.x = 0
    coverdata.y = 0
    coverdata.theta = 0

    # Open the container
    with sm: 
        StateMachine.add('MobileMotionPlanning',
                ServiceState('/plan_mobile_motion', task_assembly.srv.plan_mobile_motion,
                    request = task_assembly.srv.plan_mobile_motionRequest(mmreq.current_mobile_state,mmreq.target_mobile_pose,mmreq.Obstacles2D),
                    response_cb = mobile_fexibility_check,
                    input_keys = ['mobile_plan_res'],
                    output_keys = ['mobile_plan_traj']),
                    transitions={'succeeded':'ControllHuskyInterface'},
                    remapping={'mobile_plan_res':'mobileTraj', 
                               'mobile_plan_traj':'mobileTraj',
                               'replanning':'aborted'})

        StateMachine.add('ControllHuskyInterface',SimInterface(),
                        remapping={'simRunState':'mobileTraj'})
        # StateMachine.add('ArmMotionPlanning',
        #         ServiceState('/plan_arm_motion', task_assembly.srv.plan_mobile_motion,
        #             request = task_assembly.srv.plan_mobile_motionRequest(mmreq.current_mobile_state,mmreq.target_mobile_pose,mmreq.Obstacles2D),
        #             response_cb = arm_fexibility_check,
        #             input_keys = ['mobile_plan_res'],
        #             output_keys = ['mobile_plan_traj']),
        #             remapping={'mobile_plan_res':'mobileTraj', 
        #                        'mobile_plan_traj':'mobileTraj'
        #                        'replanning' : 'aborted'})

    # Attach a SMACH introspection server
    sis = IntrospectionServer('RL_SIM', sm, '/TAMP_start')
    sis.start()
    
    # Set preempt handler
    smach_ros.set_preempt_handler(sm)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
