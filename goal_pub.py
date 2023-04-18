#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist
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
# central axis of robot
FRONT_CAMERA_CENTRAL = -9
PI = 3.1415


cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
goal_2d_pub = rospy.Publisher('goal_2d', PoseStamped, queue_size=0)
move_base_goal_pub = rospy.Publisher(r'/move_base_simple/goal', PoseStamped, queue_size=0)
drive_bot_pub = rospy.Publisher(r'/cmd_vel', Twist, queue_size=0)

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
        
        self.velCmd = Twist()
    
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

    def drive_bot(self, x, y, angularZ):
        self.velCmd.linear.x = x 
        self.velCmd.linear.y = y 
        self.velCmd.angular.z = angularZ 

        drive_bot_pub.pub(self.velCmd) 

    def rotate(self, speed, angle, clockwise = True):
        # Create the Twist variable
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        # We won't use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            drive_bot_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Forcing our robot to stop
        vel_msg.angular.z = 0
        drive_bot_pub.publish(vel_msg)



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
            cmd = input("goal (g) drive (d) rotate (r)?") 
            if cmd == "g":
                action = input("x y yaw").split() 
                if (len(action) <= 1):
                    rbt.client.cancel_goal() 
                else:
                    x, y, yaw = action
                    # rbt.sendGoal(float(x), float(y), float(yaw)) 
                    rbt.movebase_client(x, y)
            elif cmd == "d":
                action = input("x y angularZ").split() 
                x, y, z = action 
                rbt.drive_bot(x, y, z)
            elif cmd == "r":
                angle = input("angle")
                rbt.rotate(0.2, float(angle))
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
                     print(distAndMarkers)
                     if distAndMarkers is not None:
                         dist, marker = distAndMarkers 
                         print("saw marker")
                         # if within 110 cm we will swap our state into docking mode 
                         if marker[0] == rbt.binTracker.binToGet.arucoId and dist[Z] < 110: 
                             # do some movement towards it 
                             print("Cancel Goal")
                             rbt.client.cancel_goal() 
                             rbt.state = ROBOT_STATE.DOCKING 
                 if rbt.client.get_state() == GoalStatus.SUCCEEDED:
                      rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
            if rbt.state == ROBOT_STATE.SEARCHING_FOR_BIN: 
                distAndMarkers = va.fetch_bin_distance_vec()
                if distAndMarkers is not None:
                    dist, marker = distAndMarkers
                    x, y, z = dist
                    # rotate 
                    if x < FRONT_CAMERA_CENTRAL + 20: 
                        rbt.drive_bot(0, 0, -0.3)
                    # too left turn right 
                    elif x >  FRONT_CAMERA_CENTRAL - 20: 
                        rbt.drive_bot(0, 0, 0.3)
                    else:
                        
                        rbt.drive_bot(0, 0, 3.14) 
                        rbt.state = ROBOT_STATE.DOCKING
                else: 
                    # rotate in place until you find it, if rotated 2x we will go back to goal movement 
                    pass 
            

            cnt += 1

        
        rate.sleep() 



