#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 
from actionlib_msgs.msg import GoalID

import argparse


cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
goal_2d_pub = rospy.Publisher('goal_2d', PoseStamped, queue_size=0)
move_base_goal_pub = rospy.Publisher(r'/move_base_simple/goal', PoseStamped, queue_size=0)

def send_goal(x, y, yaw): 
    rpyPose = PoseStamped() 
    rpyPose.header.frame_id = "map"
    rpyPose.header.stamp = rospy.get_rostime()
    rpyPose.pose.position.x = x
    rpyPose.pose.position.y = y
    rpyPose.pose.position.z = 0
    # double roll, pitch, yaw;
    # m.getRPY(roll, pitch, yaw);
    rpyPose.pose.orientation.x = 0
    rpyPose.pose.orientation.y = 0
    # yaw in radis
    rpyPose.pose.orientation.z = yaw
    rpyPose.pose.orientation.w = 0
    goal_2d_pub.publish(rpyPose)
    move_base_goal_pub.publish(rpyPose)


def cancel_goal():
    cancel_msg = GoalID()
    cancel_pub.publish(cancel_msg)

if __name__ == "__main__":
    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): 
        
        action = input("x y yaw").split() 
        if (len(action) == 1):
            cancel_goal() 
        else:
            x, y, yaw = action
            send_goal(float(x), float(y), float(yaw)) 

        rate.sleep() 