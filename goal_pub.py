#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 
from actionlib_msgs.msg import GoalID
from enum import Enum 
import video_aruco as va 
import sys
import binID



cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
goal_2d_pub = rospy.Publisher('goal_2d', PoseStamped, queue_size=0)
move_base_goal_pub = rospy.Publisher(r'/move_base_simple/goal', PoseStamped, queue_size=0)


class ROBOT_STATE(Enum):
    # state for idle, get new goal 
    IDLE = 0 
    # state for in middle of traveling to goal 
    TRAVELING = 1

    BRINGING_BACK = 2
    
    SEARCHING_FOR_BIN = 3

    DOCKING = 4

    DEDOCKING = 5 


class Robot: 

    def __init__(self, start_x = 0, start_y = 0, start_yaw = 0):
        self.start_x = start_x
        self.start_y = start_y 
        self.start_yaw = start_yaw
        self.state = ROBOT_STATE.IDLE   
        self.binTracker =  binID.BinTracker() 

    
    def send_goal(self, x, y, yaw): 
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

    def cancel_goal(self):
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)


    def fetch_bin(self, markerIds):
        print("found bins: ", bins) 
        bins = self.binTracker.checkIds(markerIds)
        
    



if __name__ == "__main__":
    cmd_line = sys.argv[1]  

    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10)
    rbt = Robot()

    while not rospy.is_shutdown(): 
        

        if (cmd_line is not None and cmd_line == "t"):
            action = input("x y yaw").split() 
            if (len(action) == 1):
                rbt.cancel_goal() 
            else:
                x, y, yaw = action
                rbt.send_goal(float(x), float(y), float(yaw)) 

            rate.sleep() 
        else:  
        # do some automatic stuff 
            if rbt.state == ROBOT_STATE.IDLE: 
                # find next empty bin and traverse to it 
                binId = 
