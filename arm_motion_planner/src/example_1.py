#!/usr/bin/env python

import rospy
import tf

import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import arm_motion_planner.msg
import arm_motion_planner.srv


class ArmMotionClient:
    left_group = ['Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
    right_group = ['Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

    def __init__(self):
        self.listener = tf.TransformListener()

        rospy.wait_for_service("/plan_left_arm_motion")
        self.plan_arm_motion = rospy.ServiceProxy(
            "/plan_left_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )
        self.joint_states = None
        self.left_joint_states = None
        self.right_joint_states = None

            
    def plan(self):
        current_joint = sensor_msgs.msg.JointState(
            name = self.left_group,
            position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        req = arm_motion_planner.srv.plan_arm_motionRequest()
        req.current_joint_state = current_joint
        req.current_mobile_state = geometry_msgs.msg.Pose2D(0.0, 0.0, 0.0)
        req.target_ee_pose = geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.3, 0.25, 0.834),
                orientation=geometry_msgs.msg.Quaternion(
                    -0.707, 0.0, 0.707, 0.0)
            )
        req.Pose_bound = arm_motion_planner.msg.PoseConstraint(
                constrain_pose=std_msgs.msg.Bool(False),
                position_bound_lower=geometry_msgs.msg.Point32(-100, -100, -100),
                position_bound_upper=geometry_msgs.msg.Point32(100, 100, 100),
                orientation_bound_lower=geometry_msgs.msg.Point32(-3.14, -3.14, -3.14),
                orientation_bound_upper=geometry_msgs.msg.Point32(3.14, 3.14, 3.14),
            )
        req.interpolate_path = std_msgs.msg.Bool(True)
        
        try:
            print(req)
            print("")
            resp = self.plan_arm_motion(req)
            print(resp)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


if __name__ == "__main__":
    rospy.init_node("exampleSNU")
    client = ArmMotionClient()
    
    client.plan()
    rospy.loginfo("...done!")
