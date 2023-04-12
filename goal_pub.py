#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from enum import Enum 
import video_aruco as va 
import sys
import binID



X,Y,Z = 0, 1, 2 



cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
goal_2d_pub = rospy.Publisher('goal_2d', PoseStamped, queue_size=0)
move_base_goal_pub = rospy.Publisher(r'/move_base_simple/goal', PoseStamped, queue_size=0)


class ROBOT_STATE(Enum):
    # state for idle, get new goal 
    IDLE = 0 
    # state for in middle of traveling to goal 
    FETCHING = 1

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


    
    def sendGoal(self, x, y, yaw): 
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

    def cancelGoal(self):
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)


    def fetch_bin(self, markerIds):
        print("found bins: ", bins) 
        bins = self.binTracker.checkIds(markerIds)
        
    def goalReached(self, data):
        if data[0].text == "Goal Reached":
            if self.state ==  ROBOT_STATE.FETCHING:
                self.state = ROBOT_STATE.DOCKING 
            elif self.state == ROBOT_STATE.BRINGING_BACK:
                self.state = ROBOT_STATE.DEDOCKING 


if __name__ == "__main__":
    cmd_line = sys.argv[1]  

    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10)
    rbt = Robot()
    rospy.Subscriber("/move_base/status", GoalStatusArray, rbt.goalReached)

    cnt = 0
    
    while not rospy.is_shutdown(): 
        if cnt % 10 == 0:
            print(rbt.state) 
        
        if (cmd_line is not None and cmd_line == "t"):
            action = input("x y yaw").split() 
            if (len(action) == 1):
                rbt.cancel_goal() 
            else:
                x, y, yaw = action
                rbt.send_goal(float(x), float(y), float(yaw)) 

        else:  
        # do some automatic stuff 
            if rbt.state == ROBOT_STATE.IDLE: 
                cnt = 0
                # find next empty bin and traverse to it 
                #  [(0, 0, 0.0, 0.0, 0, 0)]      
                freeBin = rbt.binTracker.getFreeBin() 
                x, y = freeBin.getXY() 
                rbt.state = ROBOT_STATE.FETCHING 
                rbt.sendGoal(x, y, 0)
            
            if rbt.state == ROBOT_STATE.FETCHING: 
                # check video frame once every 10 seconds 
                if cnt % 10 == 0: 
                    distAndMarkers = va.fetch_bin_distance_vec()
                    if distAndMarkers is not None:
                        dist, marker = distAndMarkers 
                        if marker[0] == rbt.binTracker.binToGet.arucoId and dist[Z] < 0.8: 
                            # do some movement towards it 
                            rbt.cancelGoal() 
                            rbt.state = ROBOT_STATE.DOCKING 
                # goal reach will be done throught hte callback from the subscriber 
                cnt += 1

        

        rate.sleep() 



