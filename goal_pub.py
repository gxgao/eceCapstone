#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from enum import Enum 
import video_aruco as va 
import sys
import binID


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
    
    def sendGoal(self, x, y, yaw): 
        # rpyPose = PoseStamped() 
        # rpyPose.header.frame_id = "map"
        # rpyPose.header.stamp = rospy.get_rostime()
        # rpyPose.pose.position.x = x
        # rpyPose.pose.position.y = y
        # rpyPose.pose.position.z = 0
        # # double roll, pitch, yaw;
        # # m.getRPY(roll, pitch, yaw);
        # qt = quaternion_from_euler(0, 0, yaw)
        # rpyPose.pose.orientation.x = qt[0]
        # rpyPose.pose.orientation.y = qt[1]
        # # yaw in radis
        # rpyPose.pose.orientation.z = qt[2]
        # rpyPose.pose.orientation.w = qt[3]

        rpyPose = PoseStamped() 
        rpyPose.header.frame_id = "map"
        rpyPose.header.stamp = rospy.get_rostime()
        rpyPose.pose.position.x = x
        rpyPose.pose.position.y = y
        rpyPose.pose.position.z = 0
        # double roll, pitch, yaw;
        # m.getRPY(roll, pitch, yaw);
        qt = quaternion_from_euler(0, 0, yaw)
        rpyPose.pose.orientation.x = qt[0]
        rpyPose.pose.orientation.y = qt[1]
        # yaw in radis
        rpyPose.pose.orientation.z = qt[2]
        rpyPose.pose.orientation.w = qt[3]

        # move_base_goal_pub.publish(rpyPose)

        goal_2d_pub.publish(rpyPose)
        print("Goal sent")
        
    def cancelGoal(self):
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)


    def fetch_bin(self, markerIds):
        print("found bins: ", bins) 
        bins = self.binTracker.checkIds(markerIds)
        
    def goalReached(self, data):
        if len(data.status_list) > 0 and data.status_list[0]  == 3:
            if self.state ==  ROBOT_STATE.FETCHING:
                self.state = ROBOT_STATE.DOCKING 
            elif self.state == ROBOT_STATE.BRINGING_BACK:
                self.state = ROBOT_STATE.DEDOCKING 

    def movebase_client(self, x, y):
        x = float(x)
        y = float(y)


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)



if __name__ == "__main__":
    cmd_line = sys.argv[1] if len(sys.argv) > 1 else None  

    rospy.init_node('goal_publisher')
    rate = rospy.Rate(10)
    rbt = Robot()
    rospy.Subscriber("/move_base/status", GoalStatusArray, rbt.goalReached)

    cnt = 0
    print("Starting runs!")
    while not rospy.is_shutdown(): 
        if cnt % 10 == 0:
            print(f"state:  {rbt.state}") 
        
        if (cmd_line is not None and cmd_line == "t"):
            action = input("x y yaw").split() 
            if (len(action) <= 1):
                rbt.client.cancel_goal() 
            else:
                x, y, yaw = action
                # rbt.sendGoal(float(x), float(y), float(yaw)) 
                rbt.movebase_client(x, y)
        else:  
        # do some automatic stuff 
            if rbt.state == ROBOT_STATE.IDLE: 
                cnt = 0
                # find next empty bin and traverse to it 
                #  [(0, 0, 0.0, 0.0, 0, 0)]      
                freeBin = rbt.binTracker.getFreeBin() 
                x, y = freeBin.getXY()
                print(f"free bin {float(x)}, {float(y)}")
                rbt.state = ROBOT_STATE.FETCHING 
                # rbt.sendGoal(float(x), float(y), float(0.0))
                rbt.movebase_client(x, y)
            if rbt.state == ROBOT_STATE.FETCHING: 
                 # check video frame once every 10 seconds 
                 if cnt % 10 == 0: 
                     distAndMarkers = va.fetch_bin_distance_vec()
                     if distAndMarkers is not None:
                         dist, marker = distAndMarkers 
                         print("saw marker")
                         if marker[0] == rbt.binTracker.binToGet.arucoId and dist[Z] < 0.8: 
                             # do some movement towards it 
                             print("Cancel Goal")
                             rbt.client.cancel_goal() 
                             rbt.state = ROBOT_STATE.DOCKING 
                             # goal reach will be done throught hte callback from the subscriber 
                 if rbt.client.get_state() == GoalStatus.SUCCEEDED:
                      rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
            cnt += 1

        
        rate.sleep() 



