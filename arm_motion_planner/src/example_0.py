#!/usr/bin/env python

import rospy
import tf

import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import arm_motion_planner.msg
import arm_motion_planner.srv
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionGoal
)
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ArmMotionClient:
    left_group = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
    right_group = ['Waist_Roll', 'Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

    def __init__(self):
        self.listener = tf.TransformListener()

        rospy.wait_for_service("/plan_left_arm_motion")
        self.plan_left_arm_motion = rospy.ServiceProxy(
            "/plan_left_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )
        rospy.wait_for_service("/plan_right_arm_motion")
        self.plan_right_arm_motion = rospy.ServiceProxy(
            "/plan_right_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )
        self.joint_states = None
        self.left_joint_states = None
        self.right_joint_states = None

        self.pub_obstacle_marker = rospy.Publisher('/snu_test', MarkerArray,queue_size=10)
        self.pub_motion = rospy.Publisher('/cmd_joint', sensor_msgs.msg.JointState ,queue_size=10)
        self.sub_joints = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self._callback_joints)
            
    def _callback_joints(self, data):
        self.joint_states = data
        self.left_joint_states = sensor_msgs.msg.JointState()
        self.right_joint_states = sensor_msgs.msg.JointState()

        for i in self.left_group:
            idx = data.name.index(i)
            self.left_joint_states.header = data.header
            self.left_joint_states.name.append(i)
            self.left_joint_states.position.append(data.position[idx])

        
        for i in self.right_group:
            idx = data.name.index(i)
            self.right_joint_states.header = data.header
            self.right_joint_states.name.append(i)
            self.right_joint_states.position.append(data.position[idx])
        
    def plan(self):
        # get current joint states
        current_group = self.left_group
        current_joints = None
        arm_motion_srv = None

        if current_group == self.left_group:
            arm_motion_srv = self.plan_left_arm_motion
            current_joints = self.left_joint_states
        elif current_group == self.right_group:
            arm_motion_srv = self.plan_right_arm_motion
            current_joints = self.right_joint_states

        # current joint states
        current_joint = sensor_msgs.msg.JointState(
            name = self.left_group,
            position = [0.12373800575733185, 0.1422819048166275, 2.467411518096924, 1.2070646286010742, -0.23103618621826172, 0.6453601121902466, -0.18285980820655823, 1.4761558771133423],
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
                
        req = arm_motion_planner.srv.plan_arm_motionRequest()
        req.current_joint_state = current_joint
        req.current_mobile_state = geometry_msgs.msg.Pose2D(0.0, 0.0, 0.0)

        # target gripper pose
        # req.target_ee_pose = geometry_msgs.msg.Pose(
        #         position=geometry_msgs.msg.Point(0.0, 0.6245, 0.910),
        #         orientation=geometry_msgs.msg.Quaternion(
        #             0.0, 0.707107, 0.707107, 0.0)
        #)
        req.target_ee_pose = geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.287267, 0.187007, 0.828976),
                orientation=geometry_msgs.msg.Quaternion(
                    0.612095, -0.353509, 0.612709, 0.353497)
            )

        # constraints
        req.Pose_bound = arm_motion_planner.msg.PoseConstraint(
                constrain_pose=std_msgs.msg.Bool(True),
                position_bound_lower=geometry_msgs.msg.Point32(-0.55, -0.55, -0.55),
                position_bound_upper=geometry_msgs.msg.Point32(0.55, 0.55, 0.55),
                orientation_bound_lower=geometry_msgs.msg.Point32(-0.15, -0.15, -0.15),
                orientation_bound_upper=geometry_msgs.msg.Point32(0.15, 0.15, 0.15),
        )
        req.interpolate_path = std_msgs.msg.Bool(True)

        # obstacles
        # juice
        obs1 = arm_motion_planner.msg.Obstacle3D()
        obs1.Box_dimension.x = 0.0618015/2.0
        obs1.Box_dimension.y = 0.059508/2.0
        obs1.Box_dimension.z = 0.23814/2.0
        obs1.Box_pose.position.x = 0.3
        obs1.Box_pose.position.y = 0.165 
        obs1.Box_pose.position.z = 0.82886
        obs1.Box_pose.orientation.x = 0.0
        obs1.Box_pose.orientation.y = 0.0
        obs1.Box_pose.orientation.z = 0.0
        obs1.Box_pose.orientation.w = 1.0

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
        
        # table
        obs3 = arm_motion_planner.msg.Obstacle3D()
        obs3.Box_dimension.x = 0.6
        obs3.Box_dimension.y = 0.375
        obs3.Box_dimension.z = 0.345
        obs3.Box_pose.position.x = 0.575
        obs3.Box_pose.position.y = 0.0
        obs3.Box_pose.position.z = 0.365
        obs3.Box_pose.orientation.x = 0.0
        obs3.Box_pose.orientation.y = 0.0
        obs3.Box_pose.orientation.z = 0.707107
        obs3.Box_pose.orientation.w = 0.707107

        req.Obstacles3D = [obs1, obs2, obs3]

        try:
            resp = arm_motion_srv(req)
            resp.joint_trajectory.header.stamp = rospy.Time.now()
            resp.joint_trajectory.header.frame_id = 'base_footprint'
            return resp, req.Obstacles3D
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def draw_obstacles(self, obstacles):   
        markerArray = MarkerArray()     
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "/base_footprint"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = obs.Box_dimension.x*2
            marker.scale.y = obs.Box_dimension.y*2
            marker.scale.z = obs.Box_dimension.z*2
            marker.pose.position = obs.Box_pose.position
            marker.pose.orientation = obs.Box_pose.orientation
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0)
            marker.ns = str(i)
            markerArray.markers.append(marker)
        self.pub_obstacle_marker.publish(markerArray)
        
if __name__ == "__main__":
    rospy.init_node("exampleSNU")
    client = ArmMotionClient()    
    rospy.loginfo("wait for joint states")
    # while(not client.left_joint_states):
    #     continue    
    rospy.loginfo("got the joint states")
    res, obstacles = client.plan()  
    client.draw_obstacles(obstacles)
    rospy.loginfo("motion planning is done!")
    for i, pt in enumerate(res.joint_trajectory.points):
        pt.velocities = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        pt.accelerations = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        pt.time_from_start = rospy.Duration(0.025*i)
    if(not res.joint_trajectory.points):
        rospy.loginfo("motion planning is failed.")
    else:
        # action
        ac = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        ac.wait_for_server()
        goal = FollowJointTrajectoryActionGoal().goal
        goal.trajectory = res.joint_trajectory
        rospy.loginfo("Wait until robot is stopped.")   
        
        ac.send_goal(goal)
        ac.wait_for_result()
        ac.get_result()
        rospy.loginfo("Action is finished.")

        # #topic publish
        # for pt in res.joint_trajectory.points:    
        #     joint_state = sensor_msgs.msg.JointState()
        #     joint_state.header = res.joint_trajectory.header
        #     joint_state.name = res.joint_trajectory.joint_names
        #     joint_state.position = pt.positions
        #     client.pub_motion.publish(joint_state)
        #     d = rospy.Duration(0.02)
        #     rospy.sleep(d)

        #     client.draw_obstacles(obstacles)