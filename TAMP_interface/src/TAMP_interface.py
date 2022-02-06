#!/usr/bin/env python
from operator import imod
import numpy
import rospy

import threading

import smach
import numpy
from smach import StateMachine, Concurrence
import smach_ros
import tf2_ros

from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import task_assembly.srv
from task_assembly.srv import plan_mobile_motion, plan_mobile_motionRequest, plan_arm_motionResponse
from task_assembly.srv import plan_arm_motion, plan_arm_motionRequest, plan_mobile_motionResponse
from task_assembly.srv import base_placement, base_placementRequest, base_placementResponse
from task_assembly.srv import sim_request, sim_requestRequest, sim_requestResponse

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from task_assembly.msg import GraspConfigList
from task_assembly.msg import GraspConfig
from task_assembly.msg import Obstacle2D, Obstacle3D, ObstacleBox2D
from task_assembly.msg import MobileTrajectory
from geometry_msgs.msg import Pose2D, Pose, TransformStamped
from trajectory_msgs.msg import JointTrajectory
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
                                output_keys=['targetGrasp'])

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
        userdata.targetGrasp = self.doorHandle
        # userdata.targetGrasp_out = userdata.targetGrasp_in 
        if len(self.doorHandle.grasps) <= 0:
            return "failed"
        else :
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

def base_pose_sampler(userdata,response): 
    if response.IsSucceeded.data :
        userdata.robotBasePose = response.target_mobile_pose
        userdata.sampleState.data = response.IsSucceeded.data
        userdata.targetEEpose = response.target_ee_pose
        return 'succeeded'
    else :
        return 'failed'

def mobile_fexibility_check(userdata,response): 
    # print(len(response.mobile_trajectory.points))
    userdata.mobile_plan_traj = response.mobile_trajectory
    userdata.planState = response.IsSucceeded
    if(response.IsSucceeded.data) :
        return "succeeded"
    else :
        return "replanning"

def arm_fexibility_check(userdata,response): 
    # print(len(response.mobile_trajectory.points))
    userdata.arm_trajectory_res = response.joint_trajectory
    userdata.planState = response.IsSucceeded
    if(response.IsSucceeded.data) :
        return "succeeded"
    else :
        return "replanning"

# main
def main():
    rospy.init_node('TAMP_Scence')

    # Create the top level SMACH state machine
    sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    ########################################### Define User-Data ######################################################################
    sm.userdata.simRunState = SimRunState.RUN_SIMULATION.value
    sm.userdata.armTraj = plan_arm_motionResponse()
    sm.userdata.mobileTraj = plan_mobile_motionResponse()
    sm.userdata.armTraj = JointTrajectory()
    sm.userdata.graspConfig = GraspConfigList()
    sm.userdata.robotBase = Pose2D()
    sm.userdata.TCPpose = Pose()
    sm.userdata.IsSucceeded = Bool()
    ###################################################################################################################################
    # mmreq.Obstacles2D.append(obs1)

    coverdata = Pose2D()
    coverdata.x = 0
    coverdata.y = 0
    coverdata.theta = 0

                # transitions={'failed':'SearchContinousWorkSpace',
                #              'succeeded':'basePoseSampler'}
    # Open the container
    with sm: 
        StateMachine.add('SearchContinousWorkSpace',getRandomWork(),
                transitions={'succeeded':'basePoseSampler',
                             'failed':'SearchContinousWorkSpace'},
                remapping={'targetGrasp':'graspConfig'})

        @smach.cb_interface(input_keys=['targetGraspConfig'])
        def base_pose_sampler_request_cb(userdata, request):
            srv_request = base_placementRequest()
            srv_request.minEntry.data = 10
            srv_request.eGridySearch.data = False
            srv_request.targetGrasp = userdata.targetGraspConfig

            ######### Obstacle Impormation #######################################################################################
            refrige = ObstacleBox2D()
            refrige.maxX  = 2.2250 + 0.25
            refrige.maxY  = -0.32500 + 0.25
            refrige.minX  = 2.2250 - 0.25
            refrige.minY  = -0.32500 - 0.25
            return srv_request

        @smach.cb_interface(input_keys=['mobileTarget'])
        def mobile_motion_plan_request_cb(userdata, request):
            mobile_srv_request = plan_mobile_motionRequest()

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            rate = rospy.Rate(10.0)
            trans = TransformStamped()
            while not trans.header.frame_id == "map":
                try:
                    trans = tfBuffer.lookup_transform("map", "mobile_base", rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue

            mobile_srv_request.current_mobile_state.x = trans.transform.translation.x
            mobile_srv_request.current_mobile_state.y = trans.transform.translation.y
            mobile_srv_request.current_mobile_state.theta = 0  #추후 변경예정

            mobile_srv_request.target_mobile_pose.x = userdata.mobileTarget.x
            mobile_srv_request.target_mobile_pose.y = userdata.mobileTarget.y
            mobile_srv_request.target_mobile_pose.theta = userdata.mobileTarget.theta

            # Obstacle in Given Scence
            obs1 = Obstacle2D()
            obs1.x.data = 2.2250
            obs1.y.data = -3.2500e-01
            obs1.radius.data = 0.25

            mobile_srv_request.Obstacles2D.append(obs1)

            return mobile_srv_request

        @smach.cb_interface(input_keys=['mobile_goal','targetEEpose'])
        def arm_motion_plan_request_cb(userdata, request):
            srv_request = plan_arm_motionRequest()

            userdata.targetEEpose.position.x = userdata.targetEEpose.position.x - 0.30861
            userdata.targetEEpose.position.z = userdata.targetEEpose.position.z - 0.4405
            srv_request.target_ee_pose = userdata.targetEEpose
            srv_request.current_mobile_state = userdata.mobile_goal.points[-1]

            srv_request.current_joint_state = rospy.wait_for_message("/panda/joint_states",JointState, timeout=5)
            srv_request.interpolate_path.data = True

            obs = Obstacle3D()
            obs.Box_dimension.x = 0.25
            obs.Box_dimension.y = 0.25
            obs.Box_dimension.z = 0.68
            
            obs.Box_pose.orientation.x = 0.0
            obs.Box_pose.orientation.y = 0.0
            obs.Box_pose.orientation.z = 0.0
            obs.Box_pose.orientation.w = 1.0

            srv_request.Pose_bound.position_bound_lower.x = -0.35
            srv_request.Pose_bound.position_bound_lower.y = -0.15
            srv_request.Pose_bound.position_bound_lower.z = -0.15

            srv_request.Pose_bound.position_bound_upper.x = 0.35
            srv_request.Pose_bound.position_bound_upper.y = 0.15
            srv_request.Pose_bound.position_bound_upper.z = 0.15

            srv_request.Pose_bound.orientation_bound_lower.x = -0.05
            srv_request.Pose_bound.orientation_bound_lower.y = -0.05
            srv_request.Pose_bound.orientation_bound_lower.z = -0.05

            srv_request.Pose_bound.orientation_bound_upper.x = 0.05
            srv_request.Pose_bound.orientation_bound_upper.y = 0.05
            srv_request.Pose_bound.orientation_bound_upper.z = 0.05
            return srv_request

        StateMachine.add('basePoseSampler',
                ServiceState('/plan_base_sampling', task_assembly.srv.base_placement,
                    request_cb = base_pose_sampler_request_cb,
                    response_cb = base_pose_sampler,
                    input_keys = ['targetGraspConfig','sampleState'],
                    output_keys = ['robotBasePose','targetEEpose'],
                    outcomes=['succeeded','failed']),
                    transitions={'succeeded':'MobileMotionPlanning',
                                 'failed':'SearchContinousWorkSpace'},
                    remapping={'targetGraspConfig':'graspConfig', 
                               'robotBasePose':'robotBase',
                               'sampleState':'IsSucceeded',
                               'targetEEpose':'TCPpose'})

        StateMachine.add('MobileMotionPlanning',
                ServiceState('/plan_mobile_motion', task_assembly.srv.plan_mobile_motion,
                    request_cb = mobile_motion_plan_request_cb,
                    response_cb = mobile_fexibility_check,
                    input_keys = ['mobileTarget'],
                    output_keys = ['mobile_plan_traj','planState'],
                    outcomes=['succeeded','replanning']),
                    transitions={'succeeded':'ArmMotionPlanning',
                                 'replanning':'basePoseSampler'},
                    remapping={'mobileTarget':'robotBase', 
                               'mobile_plan_traj':'mobileTraj',
                               'planState':'IsSucceeded'})

        StateMachine.add('ArmMotionPlanning',
                ServiceState('/plan_arm_motion', task_assembly.srv.plan_arm_motion,
                    request_cb = arm_motion_plan_request_cb,
                    response_cb = arm_fexibility_check,
                    input_keys = ['mobile_goal','targetEEpose'],
                    output_keys = ['arm_trajectory_res','planState'],
                    outcomes=['succeeded','replanning']),
                    transitions={'succeeded':'ControllHuskyInterface',
                                 'replanning':'basePoseSampler'},
                    remapping={'mobile_goal':'mobileTraj', 
                               'targetEEpose':'TCPpose',
                                'arm_trajectory_res':'armTraj',
                                'planState':'IsSucceeded'})

        StateMachine.add('ControllHuskyInterface',SimInterface(),
                        remapping={'simRunState':'mobileTraj'})

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
