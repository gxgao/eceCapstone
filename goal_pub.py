#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 


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

if __name__ == "__main__":
    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown(): 
        

        x, y, yaw = input("x y yaw").split() 
        
        send_goal(float(x), float(y), float(yaw)) 
        rate.sleep() 