#! /usr/bin/env python

# Getting input and sending them to
# the compute node 

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

move_group = moveit_commander.MoveGroupCommander("robot_arm")

rospy.init_node('get_input')


def random_pose():
    rand_pose = move_group.get_random_pose()
    return rand_pose.pose


def publish_pose(pose_goal=random_pose()):
    rate = rospy.Rate(0.1)
    pub = rospy.Publisher('pose_goal', Pose, queue_size=10)

    while not rospy.is_shutdown(): 
        pub.publish(pose_goal)
        print pose_goal
        rate.sleep()

def publish_random_pose():
    # Publish every 20 seconds
    rate = rospy.Rate(0.05)
    pub = rospy.Publisher('pose_goal', Pose, queue_size=10)

    while not rospy.is_shutdown(): 
        pose_goal = random_pose()
        # Check if z is less than 0
        while pose_goal.position.z < 1 or pose_goal.position.x<0:
            pose_goal = random_pose()
            
        pub.publish(pose_goal)
        print pose_goal
        rate.sleep()
 
def main():
    publish_random_pose()

if __name__ == '__main__':
    main()
