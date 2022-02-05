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
from task_assembly.srv import plan_arm_motion, plan_arm_motionRequest
from task_assembly.msg import Obstacle2D
from task_assembly.msg import MobileTrajectory
from geometry_msgs.msg import Pose2D
from math import pi

# define state EatCookie
class MobileMotionContrainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['excution'],
                             input_keys=['mobile_trajectory'],
                             output_keys=['sim_excute'])
        
        self.data = MobileTrajectory()
        self.counter = 0

    def execute(self, userdata):
        print("-----------------------------------------------------")
        self.data = userdata.mobile_trajectory
        rospy.loginfo('Executing state MobileMotionContrainer')
        return 'excution'

# main
def main():
    rospy.init_node('TAMP_Scence')

    # Create the top level SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    
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
    sm0.userdata.sim_excute = 0
    sm0.userdata.mobile_trajectory = [coverdata]
    # sm0.
    # Open the container
    with sm0:
        def gripper_request_cb(userdata, request):
            gripper_request = plan_mobile_motion().Request
            gripper_request.position.x = 2.0
            gripper_request.max_effort = userdata.gripper_input
            return gripper_request
        
        smach.StateMachine.add('MobileMotionContrainer', MobileMotionContrainer(), 
                        transitions={'excution':'MobileMotion'})
                                    
        StateMachine.add('MobileMotion',
                ServiceState('/plan_mobile_motion', task_assembly.srv.plan_mobile_motion,
                    request = task_assembly.srv.plan_mobile_motionRequest(mmreq.current_mobile_state,mmreq.target_mobile_pose,mmreq.Obstacles2D),
                    request_slots = ['mobile_trajectory'],
                    output_keys = ['mobile_trajectory']),
                    transitions={'succeeded':'MobileMotionContrainer'})

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/TAMP')
    sis.start()
    
    # Set preempt handler
    smach_ros.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
