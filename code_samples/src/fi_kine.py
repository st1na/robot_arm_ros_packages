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

class Kinematics:
    def __init__(self):
        print("Kinematics Tutorial")

    def fkine(self, joint_goal):
        joint_goal_v = move_group.get_current_joint_values()
        joint_goal_v[0] = joint_goal[0]
        # Second real joint
        joint_goal_v[1] = joint_goal[1]
        #Third real joint
        joint_goal_v[2] = joint_goal[2]
        #Fourth real joint
        joint_goal_v[5] = joint_goal[3]
        #Fifth real joint
        joint_goal_v[6] = joint_goal[4] 

        # joint_gol 3 and 4 are virtual
        joint_goal_v[3] = 0   
        joint_goal_v[4] = 0
        # joint 7 and 8 are virtual
        joint_goal_v[7] = 0
        joint_goal_v[8] = 0
        move_group.go(joint_goal_v, wait=True)
        move_group.stop()
        ''' print "Desired Joint values are: "
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]
        print move_group.get_current_joint_values()[5]
        print move_group.get_current_joint_values()[6]
        '''
        print "Pose is: "
        print move_group.get_current_pose().pose
        
  
    def ikine(self, pose_goal):
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)
        move_group.stop()
        #print "Desired Pose is: "
        #print move_group.get_current_pose().pose
        print "Joint values are: "
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]
        print move_group.get_current_joint_values()[5]
        print move_group.get_current_joint_values()[6]

def main():
    rospy.init_node('fi_kine')
    kine = Kinematics()
    joint_goal = [pi/2, pi/2, pi/2, pi/2, pi/2]
    print "============ Press `Enter` to execute a movement using a joint state goal : "
    print joint_goal
    raw_input()
    kine.fkine(joint_goal)

    #rand_pose = move_group.get_random_pose()
    print "==============================================================================="
    print "============ Press `Enter` to execute a movement using a pose goal :"
    #print rand_pose.pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = -0.8626
    pose_goal.orientation.y = -0.4183
    pose_goal.orientation.z =  0.1779
    pose_goal.orientation.w = 0.2218
    pose_goal.position.x = -1.9131
    pose_goal.position.y = 1.0260
    pose_goal.position.z = -0.0542
    print pose_goal
    raw_input()

    kine.ikine(pose_goal)
    

if __name__ == '__main__':
    main()
