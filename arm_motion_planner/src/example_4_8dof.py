#!/usr/bin/env python

import rospy
import tf
import PyKDL

import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import arm_motion_planner.msg
import arm_motion_planner.srv
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionGoal
)

class ArmMotionClient:
    left_group = ['Waist_Roll','Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
    right_group = ['Waist_Roll','Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

    def __init__(self):
        self.listener = tf.TransformListener()

        rospy.wait_for_service("/plan_left_arm_motion")
        self.plan_arm_motion = rospy.ServiceProxy(
            "/plan_left_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )
        self.joint_states = None
        self.left_joint_states = None
        self.right_joint_states = None

        self.pub_motion = rospy.Publisher('/cmd_joint', sensor_msgs.msg.JointState ,queue_size=10)

            
    def plan(self):
        # current joint states
        current_joint = sensor_msgs.msg.JointState(
            name = self.left_group,
            position = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velocity = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            effort = [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
                
        req = arm_motion_planner.srv.plan_arm_motionRequest()
        req.current_joint_state = current_joint
        req.current_mobile_state = geometry_msgs.msg.Pose2D(0.0, 0.0, 0.0)

        # target gripper pose
        # req.target_ee_pose = geometry_msgs.msg.Pose(
        #         position=geometry_msgs.msg.Point(0.287177, 0.186931, 0.8288),
        #         orientation=geometry_msgs.msg.Quaternion(
        #             0.612076, -0.353723, 0.612333, 0.353965)
        #     )
        req.target_ee_pose = geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.3, 0.18, 0.828976),
                orientation=geometry_msgs.msg.Quaternion(
                    0.612095, -0.353509, 0.612709, 0.353497)
            )
        # constraints
        req.Pose_bound = arm_motion_planner.msg.PoseConstraint(
                constrain_pose=std_msgs.msg.Bool(False),
                position_bound_lower=geometry_msgs.msg.Point32(-0.35, -0.15, -0.15),
                position_bound_upper=geometry_msgs.msg.Point32(0.35, 0.15, 0.15),
                orientation_bound_lower=geometry_msgs.msg.Point32(-0.05, -0.05, -0.05),
                orientation_bound_upper=geometry_msgs.msg.Point32(0.05, 0.05, 0.05),
            )
        req.interpolate_path = std_msgs.msg.Bool(False)

        # obstacles
        # juice
        
        obs1 = arm_motion_planner.msg.Obstacle3D()
        obs1.Box_dimension.x = 8.2402e-02/2.0
        obs1.Box_dimension.y = 7.9344e-02/2.0
        obs1.Box_dimension.z = 0.23814/2.0
        obs1.Box_pose.position.x = 0.35
        obs1.Box_pose.position.y = 0.115 
        obs1.Box_pose.position.z = 0.82886
        obs1.Box_pose.orientation.x = 0.0
        obs1.Box_pose.orientation.y = 0.0
        obs1.Box_pose.orientation.z = 0.0
        obs1.Box_pose.orientation.w = 1.0
        """
        # milk
        obs2 = arm_motion_planner.msg.Obstacle3D()
        obs2.Box_dimension.x = 0.065/2.0 
        obs2.Box_dimension.y = 0.065/2.0 
        obs2.Box_dimension.z = 0.23544/2.0
        obs2.Box_pose.position.x = 0.325 
        obs2.Box_pose.position.y = 0.0 
        obs2.Box_pose.position.z = 0.8275
        obs2.Box_pose.orientation.x = 0.0
        obs2.Box_pose.orientation.y = 0.0
        obs2.Box_pose.orientation.z = 0.707107
        obs2.Box_pose.orientation.w = 0.707107
        """
        # table
        obs3 = arm_motion_planner.msg.Obstacle3D()
        obs3.Box_dimension.x = 0.375
        obs3.Box_dimension.y = 0.6
        obs3.Box_dimension.z = 0.345
        obs3.Box_pose.position.x = 0.575
        obs3.Box_pose.position.y = 0.0
        obs3.Box_pose.position.z = 0.365
        obs3.Box_pose.orientation.x = 0.0
        obs3.Box_pose.orientation.y = 0.0
        obs3.Box_pose.orientation.z = 0.0
        obs3.Box_pose.orientation.w = 0.1
        
        req.Obstacles3D = [obs1, obs3]
        
        try:
            resp = self.plan_arm_motion(req)
            resp.joint_trajectory.header.stamp = rospy.Time.now()
            resp.joint_trajectory.header.frame_id = 'base_footprint'
            return resp
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


if __name__ == "__main__":
    rospy.init_node("exampleSNU")
    client = ArmMotionClient()    
    res = client.plan()
    #print(res)
    rospy.loginfo("motion planning is done!")

    # # action
    # ac = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # ac.wait_for_server()
    # goal = FollowJointTrajectoryActionGoal().goal
    # goal.trajectory = res.joint_trajectory
    # rospy.loginfo("Wait until robot is stopped.")

    # for pt in goal.trajectory.points:
    #     pt.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #     pt.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #     pt.time_from_start = rospy.Duration(0.03)
    
    # print(goal)
    # ac.send_goal(goal)
    # ac.wait_for_result()
    # ac.get_result()
    # rospy.loginfo("Action is finished.")

    #topic publish
    for pt in res.joint_trajectory.points:
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header = res.joint_trajectory.header
        joint_state.name = res.joint_trajectory.joint_names
        joint_state.position = pt.positions
        client.pub_motion.publish(joint_state)
        d = rospy.Duration(0.15)
        rospy.sleep(d)