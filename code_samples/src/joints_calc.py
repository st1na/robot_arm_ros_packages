#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import math
from math import pi



robot = moveit_commander.RobotCommander()
#scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("robot_arm")


class PoseState:
    def __init__(self):
        self.pose_goal_sub = rospy.Subscriber('pose_goal', geometry_msgs.msg.Pose, self.pose_callback)

    def pose_callback(self, pose_goal):
        self.compute_joints(pose_goal)

    def compute_joints(self, pose_goal):
        # compute the joints from the pose
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_joint_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)
        move_group.set_planning_time(5.0)

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()

class JointState:
    GRIPPER_OPEN   = 1
    GRIPPER_CLOSE  = 0
    
    # Subscribe to joins_states
    # robot_joints hold the joints needed to be send to the
    # sending node
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.joint_callback)
        self.robot_joints = sensor_msgs.msg.JointState

    def joint_callback(self, joint_states):
        self.robot_joints = self.joint_remove_virtual(joint_states)
        self.robot_joints = self.joint_map(self.robot_joints)
        self.robot_joints = self.joint_radtodeg(self.robot_joints)
        # Send joints in range [-90, 90] deg
        send_joints(self.robot_joints)

    def joint_remove_virtual(self, joint_states):
        # joint_states name and position have
        # the same indexes, get the virtual joints
        # indexes and remove them
        virtual_joints_names = ['link2_bar1', 'link2_bar1_bar2', 'vj0', 'vj1', 'gripper_left']
        joint_states.position = list(joint_states.position)
        for i in virtual_joints_names:
            virtual_joint_index = joint_states.name.index(i)
            del joint_states.name[virtual_joint_index]
            del joint_states.position[virtual_joint_index]
        
        return joint_states

    def open_gripper(self):
        gripper_index = joint_states.name.index('gripper')
        self.robot_joints.position[gripper_index] = GRIPPER_OPEN
    
    def close_gripper(self):
        gripper_index = joint_states.name.index('gripper')
        self.robot_joints.position[gripper_index] = GRIPPER_CLOSE

    def joint_map(self, joints):
        target_min = -pi/2
        target_max = pi/2
        # Map joints to range [-pi/2, pi/2]
        # From the urdf only the second joint
        # is in that range so map only that joint
        range_min = 0
        range_max = pi
        # Map from zero to range_max
        m = joints.position[1]
        m = m - range_min
        # Map to [0,1]
        m = (m - range_min)/(range_max - range_min)
        # Map to [0, target_max]
        m = m*(target_max - target_min)
        # Map to [target_min, target_max]
        m = m + target_min
        joints.position[1] = m
        return joints

    def joint_radtodeg(self, joints):
        # Covert from radians to degrees
        for i in range(len(joints.position)):
            joints.position[i] = math.degrees(joints.position[i])
        return joints

def send_joints(goal_joints):
    goal_joints_pub = rospy.Publisher('goal_joints',sensor_msgs.msg.JointState, queue_size=10)
    goal_joints_pub.publish(goal_joints)

def main():
    rospy.init_node('joints_calc')
    pose_state = PoseState()
    joint_state = JointState()
    rospy.spin()
"""
    try:
        rospy.spin()
    except KeyboardIntterupt:
        print "Shutting down Joints Calculate node"
"""

if __name__ == '__main__':
    main()
