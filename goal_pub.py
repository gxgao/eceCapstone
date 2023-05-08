#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from enum import Enum
import video_aruco as va
import sys
import binID
import time
from lift import Arms
from math import sin, cos
from dataclasses import dataclass
import actionlib
import pdb

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

X, Y, Z = 0, 1, 2
# central axis of robot
PI = 3.1415
ENABLE_DOCKING_PRINTING = False

class Command(Enum):
    Goal = 0
    Drive = 1
    DetectBin = 2
    DriveAngle = 3
    BatteryTest = 4
    RotateOld = 5
    Rotate = 6
    SetPosition = 7
    RaiseArms = 8
    LowerArms = 9
    CancelGoal = 10
    SetRobotState = 11
    SetPickupState = 12
    SetGoalBin = 13
    GoToGoalBin = 14
    SetArmState = 15
    GoToTakeOut = 16
    GoToHome = 17
    EnterLoop = 18

@dataclass
class Vector:
    z: int
    l_x: int
    r_x: int
    speed: float
    dist: float

# TODO: add array for slim side
back_camera_array = [
    Vector(z=105, l_x=-48, r_x=-31, speed=-.1, dist=.1),
    Vector(z=85, l_x=-35, r_x=-27, speed=-.1, dist=.1),
    Vector(z=67, l_x=-31, r_x=-25, speed=-.04, dist=.04),
    Vector(z=56, l_x=-26, r_x=-24, speed=-.04, dist=.04),
    Vector(z=52, l_x=-25.5, r_x=-21.5, speed=-.04, dist=.04)
]

cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
goal_2d_pub = rospy.Publisher("goal_2d", PoseStamped, queue_size=0)
init_2d_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
move_base_goal_pub = rospy.Publisher(
    r"/move_base_simple/goal", PoseStamped, queue_size=0
)
drive_bot_pub = rospy.Publisher(r"/cmd_vel", Twist, queue_size=0)


class ROBOT_STATE(Enum):
    # state for idle, get new goal
    IDLE = 0
    # state for in middle of traveling to goal
    FETCHING = 1

    BRINGING_BACK = 2

    SEARCHING_FOR_BIN = 3

    DOCKING = 4

    RETURNING = 5

    DEDOCKING = 6

class PICKUP_STATE(Enum):
    PICKUP = 0
    RETURNING = 1

class Robot:
    def __init__(self, start_x=0, start_y=0, start_yaw=0):
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw
        self.state = ROBOT_STATE.IDLE
        self.binTracker = binID.BinTracker()

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.goal_bin_id = -1
        self.goal_bin = None 
        self.laser_ranges = []

        self.arms = Arms()

        self.cnt = 0
        self.rate = rospy.Rate(10)

        self.running = True

        self.pos = (0, 0)
        self.rot = 0
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(
            "/map", "/base_link", rospy.Time(0), rospy.Duration(3.0)
        )

        self.already_slept = False

        self.search_rotation_count = 0

        self.pickup_state = PICKUP_STATE.PICKUP

        print(f"Inital Robot State: {self.state}")

    def SetState(self, new_state):
        if self.state != ROBOT_STATE.SEARCHING_FOR_BIN and new_state == ROBOT_STATE.SEARCHING_FOR_BIN:
            self.search_rotation_count = 0
        print(f"\nSwitching State:\n{self.state} -> {new_state}\n")
        self.state = new_state

    def SetPosition(self, x, y, yaw):
        rpyPose = self.CreatePoseStamped(x, y, yaw)
        rpyPoseWithCovarianceStamped = PoseWithCovarianceStamped()
        rpyPoseWithCovarianceStamped.header.frame_id = rpyPose.header.frame_id
        rpyPoseWithCovarianceStamped.header.stamp = rpyPose.header.stamp
        rpyPoseWithCovarianceStamped.pose.pose.position = rpyPose.pose.position
        rpyPoseWithCovarianceStamped.pose.pose.orientation = rpyPose.pose.orientation

        init_2d_pub.publish(rpyPoseWithCovarianceStamped)

    def get_dropoff(self):
        return 0, 0

    def CreatePoseStamped(self, x, y, yaw):
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
        return rpyPose

    def sendGoal(self, x, y, yaw):
        rpyPose = self.CreatePoseStamped(x, y, yaw)
        goal_2d_pub.publish(rpyPose)
        print("Goal sent")

    def cancelGoal(self):
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)

    def fetch_bin(self, markerIds):
        print("found bins: ", bins)
        bins = self.binTracker.checkIds(markerIds)

    def goalReached(self, data):
        if len(data.status_list) > 0 and data.status_list[0] == 3:
            if self.state == ROBOT_STATE.FETCHING:
                self.SetState(ROBOT_STATE.DOCKING)
            elif self.state == ROBOT_STATE.BRINGING_BACK:
                self.SetState(ROBOT_STATE.DEDOCKING)

    def movebase_client(self, x, y, angle=0):
        x = float(x)
        y = float(y)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        qt = quaternion_from_euler(0, 0, angle)
        goal.target_pose.pose.orientation.x = qt[0]
        goal.target_pose.pose.orientation.y = qt[1]
        goal.target_pose.pose.orientation.z = qt[2]
        goal.target_pose.pose.orientation.w = qt[3]

        self.client.send_goal(goal)

    def drive_back(self, speedX, x, splits=20):
        dist_per_split = x / splits
        print("dist per split", dist_per_split)

        flip = True
        top = [6, 5]
        bottom = [-5, -6]
        for i in range(splits):
            if flip:
                angle = top[0]
                top.reverse()
            else:
                angle = bottom[0]
                bottom.reverse()

            flip = not flip
            self.drive_bot(-speedX, dist_per_split, angle, sleep_extra=True)

    def drive_bot(self, speedX, x, angle=None, sleep_extra=False):
        speedX, x = float(speedX), float(x)

        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = float(angle) * 2 * PI / 360 if angle is not None else 0

        # We won't use linear components
        vel_msg.linear.x = speedX
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed

        # Calculate the distance traveled
        distance = 0.0
        while distance < abs(x):
            # Publish the velocity message
            drive_bot_pub.publish(vel_msg)

            if sleep_extra:
                sleep_time = .08
                self.ForceSleepSec(sleep_time)
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                drive_bot_pub.publish(vel_msg)
                vel_msg.linear.x = speedX
                vel_msg.angular.z = angular_speed
                distance += abs(vel_msg.linear.x) * sleep_time*2
            else:
                self.ForceSleep()
                # Calculate the distance traveled
                distance += abs(vel_msg.linear.x) * self.rate.sleep_dur.to_sec()
                # Sleep to maintain the publish rate

        # Stop the robot
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0

        drive_bot_pub.publish(vel_msg)


    def laser_callback(self, data):
        self.laser_ranges = list(data.ranges)

    def rotate(self, speed, angle, clockwise=True):
        # Create the Twist variable
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed * 2 * PI / 360
        relative_angle = angle * 2 * PI / 360

        # We won't use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
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

        while current_angle < relative_angle:
            drive_bot_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        # Forcing our robot to stop
        vel_msg.angular.z = 0
        drive_bot_pub.publish(vel_msg)

    def rotate_new(self, speed, goal_angle, sleep_extra=False):
        # Create the Twist variable
        if goal_angle > 180:
            goal_angle = -180 + (goal_angle - 180)
        if goal_angle < -180:
            goal_angle = 180 - (goal_angle + 180)
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed * 2 * PI / 360

        # We won't use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        while abs(self.rot - goal_angle) > 1.35:
            if (goal_angle >= 0 and self.rot >= 0) or (goal_angle <= 0 and self.rot <= 0):
                clockwise = goal_angle >= self.rot
            elif self.rot <= 0 and goal_angle >= 0:
                clockwise = goal_angle - self.rot <= (180 + self.rot) + 180 - goal_angle
            elif self.rot >= 0 and goal_angle <= 0:
                clockwise = (180 - self.rot) + (180 + goal_angle) <= self.rot - goal_angle
            else:
                print("uh o oo")

            if clockwise:
                vel_msg.angular.z = abs(angular_speed)
            else:
                vel_msg.angular.z = -abs(angular_speed)

            drive_bot_pub.publish(vel_msg)

            if sleep_extra:
                self.ForceSleepSec(0.05)
                vel_msg.angular.z = 0
                drive_bot_pub.publish(vel_msg)
            else:
                self.ForceSleep()
            self.update_tf()

        vel_msg.angular.z = 0
        drive_bot_pub.publish(vel_msg)

    def update_tf(self):
        t = self.tf_listener.getLatestCommonTime("map", "base_link")
        position, quaternion = self.tf_listener.lookupTransform("map", "base_link", t)
        self.pos = position[:2]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.rot = yaw * 180 / 3.1415926

    def SetBinHelper(self, bin_id):
        bin_to_get = self.binTracker.SetBin(bin_id)
        if bin_to_get is None:
            print("Failed to set bin: {bin_id}")
            return
        self.goal_bin = bin_to_get
        self.goal_bin_id = bin_to_get.binId
        

    def HandleIdle(self):
        if self.pickup_state == PICKUP_STATE.PICKUP:
            bin_to_get = self.binTracker.getFreeBin()
            if bin_to_get is None:
                self.pickup_state = PICKUP_STATE.RETURNING
                input("Press Enter to switch state to RETURNING.")
                return
        elif self.pickup_state == PICKUP_STATE.RETURNING:
            bin_to_get = self.binTracker.getTakeOutBin()
            if bin_to_get is None:
                input("Press Enter to switch state to PICKUP.")
                self.pickup_state = PICKUP_STATE.PICKUP
                return
        
        x, y = bin_to_get.getXY()
        self.goal_bin = bin_to_get 
        print(f"Goal bin set at position (x,y): {float(x)}, {float(y)}")
        self.SetState(ROBOT_STATE.FETCHING)
        # rbt.sendGoal(float(x), float(y), float(0.0))
        self.movebase_client(x, y)
        self.goal_bin_id = bin_to_get.binId

    def HandleFetching(self):
        if self.cnt % 10 == 0:
            for forward in (False, True):
                distAndMarkers = self.DetectBin(forward=forward)
                if distAndMarkers is not None:
                    dist, _ = distAndMarkers
                    if dist[Z] < 110:
                        self.SetState(ROBOT_STATE.SEARCHING_FOR_BIN)
                        self.client.cancel_goal()
                        return
            
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            print("Reached destination")
            self.SetState(ROBOT_STATE.SEARCHING_FOR_BIN)
    
    def DetectBin(self, forward=True): 
        for _ in range(5):
            distAndMarkers = va.fetch_bin_distance_vec_multi(forward) 
            if distAndMarkers is None:
                return None

            distAndMarkers_cleaned = []
            sides = []
            
            for distAndMarker in zip(distAndMarkers[0], distAndMarkers[1]):
                dist, marker = distAndMarker[0][0], distAndMarker[1][0]
                if self.binTracker.ArucoIdToBinId(marker)  == self.goal_bin_id and marker % 3 != 2:
                    distAndMarkers_cleaned.append((dist, marker))
                    sides.append(self.binTracker.GetBinSide(marker))

            if len(distAndMarkers_cleaned) == 0:
                return None

            if len(distAndMarkers_cleaned) == 1:
                return distAndMarkers_cleaned[0]
            
            assert len(distAndMarkers_cleaned) == 2

            avg_z_dist = (distAndMarkers_cleaned[0][0][Z] + distAndMarkers_cleaned[1][0][Z]) / 2
            avg_x_dist = (distAndMarkers_cleaned[0][0][X] + distAndMarkers_cleaned[1][0][X]) / 2
            return [avg_x_dist, 0, avg_z_dist], self.goal_bin_id * 3 + 2

        return None 
        
    def DetectBinOrientation(self, forward=True):
        distAndMarkers = self.DetectBin()
        if distAndMarkers is not None:
            _, markers = distAndMarkers
            return self.binTracker.GetBinSide(markers[0][0])
        return None

    def HandleSearchingForBin(self):
        if (self.DetectBin(forward=False)):
            self.SetState(ROBOT_STATE.DOCKING)
            return 
 
        distAndMarker = self.DetectBin()
        

        if distAndMarker is not None:

            # print(distAndMarker)
            dist, marker = distAndMarker
            # print(
            #     "marker detected while searching using front camera", marker
            # )
            # print(self.goal_bin_id)
            # x axis is horizontal to robot, right is decreasing
            # moving roomba up increases y
            # z is obvious:

            x, _, z = dist
            # print("z is ", z)
            if z <= 30:
                self.drive_bot(-0.1, 0.1)
            elif z >= 100:
                self.drive_bot(0.1, 0.1)
            elif x < -40:
                self.rotate_new(8, self.rot + 4)
            elif x > -20:
                self.rotate_new(8, self.rot - 4)
            else:
                self.rotate_new(15, self.rot + 180)
                self.SetState(ROBOT_STATE.DOCKING)
        else:
            if self.cnt % 10 != 0:
                return

            if self.search_rotation_count > 15:
                # we lost the bin
                print(f"Lost bin with id: {self.goal_bin_id}")
                self.binTracker.SetMissing(self.goal_bin_id, True)
                self.SetState(ROBOT_STATE.IDLE)
                return
            
            self.search_rotation_count += 1
            # rotate in place until you find it, if rotated 2x we will go back to goal movement
            self.rotate(8, 25)

    def ShouldLoop(self):
        return self.running and not rospy.is_shutdown()

    def HandleDocking(self):
        ROTATION_SPEED = 5.5
        DOCKING_SPEED = -.05
        FINAL_DOCKING_SPEED = -0.1

        distAndMarker = self.DetectBin(forward=False) 

        if (distAndMarker is None):
            self.SetState(ROBOT_STATE.SEARCHING_FOR_BIN)
            return 

        dist, marker = distAndMarker
        if ENABLE_DOCKING_PRINTING:
            print("marker detected", marker)
        # get distance of bin window from lidar
        x, _, z = dist
        # dist, l_x, r_x

        # todo add speed and rotation to back camera array
        # (40, -21, -16), (31, -18, -13)
        calibrate = None
        sleep_extra = False

        for i in range(len(back_camera_array)):
            if z > back_camera_array[i].z:
                calibrate = back_camera_array[i]
                if i >= len(back_camera_array) - 2:
                    sleep_extra = True
                break
        if ENABLE_DOCKING_PRINTING:
            print(f"picked {calibrate} and sleep_extra={sleep_extra}")
        
        
        def lerp(x1, y1, x2, y2, x):
            return y1 + (y2 - y1) / (x2 - x1) * (x - x1)


        side = self.binTracker.GetBinSide(marker)
        if side == binID.BIN_SIDE.WIDE:
            aruco_z_limit = 52
        elif side == binID.BIN_SIDE.SLIM:
            aruco_z_limit = 42
        elif side == binID.BIN_SIDE.DIAGONAL:
            aruco_z_limit = 50
        
        if ENABLE_DOCKING_PRINTING:
            print(f"Step: {x=} {z=} {sleep_extra=} {marker=}")
        if z >= aruco_z_limit:
            # if not last range, we look at prev indx and do linear 
            # interpolation to find what range we should be within 
            if i == 0: 
                prev_calibrate = back_camera_array[0]
                calibrate = back_camera_array[1]
            elif calibrate == None or i == len(back_camera_array):
                prev_calibrate = back_camera_array[-2]
                calibrate = back_camera_array[-1]
            else:
                prev_calibrate = back_camera_array[i - 1]
            
            l_x = lerp(prev_calibrate.z, prev_calibrate.l_x, calibrate.z, calibrate.l_x, z) 
            r_x = lerp(prev_calibrate.z, prev_calibrate.r_x, calibrate.z, calibrate.r_x, z) 

            if x < l_x:
                if ENABLE_DOCKING_PRINTING:
                    print(f"{x} < {l_x}")
                self.rotate_new(ROTATION_SPEED, self.rot - 1.5, sleep_extra)
            elif x > r_x:
                if ENABLE_DOCKING_PRINTING:
                    print(f"{x} > {r_x}")
                self.rotate_new(ROTATION_SPEED, self.rot + 1.5, sleep_extra)
            else:
                if ENABLE_DOCKING_PRINTING:
                    print(f"{l_x} < {x} < {r_x}")
                self.drive_bot(
                    calibrate.speed, calibrate.dist
                )  # i hope since we sleep min_dist gets edited
            self.ForceSleepSec(.25)
        else:
            prev_calibrate = back_camera_array[-2]
            calibrate = back_camera_array[-1]
            l_x = lerp(prev_calibrate.z, prev_calibrate.l_x, calibrate.z, calibrate.l_x, z) 
            r_x = lerp(prev_calibrate.z, prev_calibrate.r_x, calibrate.z, calibrate.r_x, z) 
            if x < l_x or x > r_x:
                if ENABLE_DOCKING_PRINTING:
                    print("Too close not aligned. Backing out.")
                self.drive_bot(-back_camera_array[-1].speed, back_camera_array[-1].dist)
                return
            
            def get_min_dist():
                rear_laser_ranges = self.laser_ranges[500:650]
                min_dist = min(rear_laser_ranges)
                return min_dist


            if side == binID.BIN_SIDE.WIDE:
                min_dist_limit = .27 
            elif side == binID.BIN_SIDE.SLIM:
                min_dist_limit = .20
            elif side == binID.BIN_SIDE.DIAGONAL:
                min_dist_limit = .22
            

            distances = []
            while get_min_dist() > min_dist_limit:
                min_dist = get_min_dist()
                distances.append(min_dist)
                dist_to_travel = min(0.03, max(.02, min_dist - min_dist_limit))
                if ENABLE_DOCKING_PRINTING:
                    print("min_dist ", min_dist)
                    print("dist to travel ", dist_to_travel)
                # self.drive_back(0.05, dist_to_travel)
                self.drive_bot(DOCKING_SPEED, dist_to_travel)
                self.ForceSleepSec(1.0)

                if len(distances) > 3 and abs(distances[-1] - distances[-3]) < .01:
                    break
                prev_min_dist = min_dist
            if ENABLE_DOCKING_PRINTING:
                print("why are we stopping early", get_min_dist(), min_dist_limit)
            # add lift code ...
            if get_min_dist() <= min_dist_limit + .05:
                # post lift code
                # self.movebase_client(*self.get_dropoff())
                # self.state = ROBOT_STATE.RETURNING
                self.drive_bot(FINAL_DOCKING_SPEED, .06)
                self.ForceSleepSec(2)
                self.arms.raise_arms()
                # print("done")
                self.binTracker.binPickUp(self.goal_bin)
                self.goal_bin = None 
                x, y, angle = self.binTracker.currBin.getTakeOutPos()
                # if this bin has already been taken out we want the goal to be the home 
                if not self.binTracker.currBin.takeOut: 
                    x, y, angle = self.binTracker.currBin.getHomePos() 
                self.movebase_client(x, y, angle)
                self.SetState(ROBOT_STATE.RETURNING)
                # self.drive_bot(-.05, .2)
                # self.ForceSleepSec(6)
                # self.arms.lower_arms()
            else:
                self.running = False
                print("Failed to pickup")

    def HandleReturning(self):
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.SetState(ROBOT_STATE.DEDOCKING)
    
    def HandleDedocking(self):

        rob_x, rob_y = self.pos
        if False:
            arm_offset = 0.2794
            ang = self.rot
            arm_ang = (ang + 180) % 360
            arm_ang = arm_ang if arm_ang <= 180 else arm_ang - 360
            arm_ang = arm_ang * 3.1415926 / 180
            # arm_ang 
            # TODO change to rad b4 put in db
            bin_x = arm_offset * cos(arm_ang) + rob_x
            bin_y = arm_offset * sin(arm_ang) + rob_y
        # flip the takeout status 
        self.binTracker.binDrop(not self.binTracker.currBin.takeOut, rob_x, rob_y, 0)

        self.SetState(ROBOT_STATE.IDLE)
        self.arms.lower_arms()

        self.drive_bot(.05, .5)

    def RunLoopOnce(self):
        if self.state == ROBOT_STATE.IDLE:
            self.HandleIdle()
        elif self.state == ROBOT_STATE.FETCHING:
            self.HandleFetching()
        elif self.state == ROBOT_STATE.SEARCHING_FOR_BIN:
            self.HandleSearchingForBin()
        elif self.state == ROBOT_STATE.DOCKING:
            self.HandleDocking()
        elif self.state == ROBOT_STATE.RETURNING:
            self.HandleReturning()
        elif self.state == ROBOT_STATE.DEDOCKING:
            self.HandleDedocking()
        else:
            print("Unkown State: ", self.state)

    def Sleep(self):
        if not self.already_slept:
            self.rate.sleep()
            self.already_slept = True

    def ForceSleep(self):
        self.already_slept = True 
        self.rate.sleep()
    
    def ForceSleepSec(self, sec):
        if sec >= .1:
            self.already_slept = True 
        time.sleep(sec)

    def BeforeEveryLoop(self):
        self.cnt += 1 
        self.update_tf()
        self.already_slept = False

    def Loop(self):
        while self.ShouldLoop():
            self.BeforeEveryLoop()
            self.RunLoopOnce()
            self.Sleep()
         

    def HandleManual(self):
        while True:
            cmd = input("Input Command Number. Enter 'h' for Command Enum values: ")
            
            if cmd == "h":
                for command in Command:
                    print(f"{command.value}: {command.name}")
                continue
                
            cmd = Command(int(cmd))
            if cmd == Command.Goal:
                action = input("x y yaw: ").split()
                if len(action) <= 1:
                    self.client.cancel_goal()
                else:
                    x, y, yaw = action
                    # rbt.sendGoal(float(x), float(y), float(yaw))
                    self.movebase_client(x, y)
            # elif cmd
            elif cmd == Command.Drive:
                action = input("speed x, x: ").split()
                speedx, x = action
                self.drive_bot(speedx, x)
            elif cmd == Command.DetectBin:
                print(self.DetectBin())
            elif cmd == Command.DriveAngle:
                action = input("speed x, x, a: ").split()
                speedx, x, a = action
                self.drive_bot(speedx, x, a)
            elif cmd == Command.BatteryTest:
                while True:
                    self.movebase_client(2.5, 0)
                    time.sleep(20)
                    self.arms.raise_arms()
                    time.sleep(2)
                    self.arms.lower_arms()
                    time.sleep(2)
                    self.movebase_client(0, 0)
                    time.sleep(20)
            elif cmd == Command.RotateOld:
                angle = input("angle: ")
                self.rotate(30, float(angle))
            elif cmd == Command.Rotate:
                angle = input("angle: ")
                self.rotate_new(5.5, float(angle))
            elif cmd == Command.SetPosition:
                x, y, yaw = input("x, y, yaw: ").split()
                self.SetPosition(float(x), float(y), float(yaw))
            elif cmd == Command.RaiseArms:
                self.arms.raise_arms()
            elif cmd == Command.LowerArms:
                self.arms.lower_arms()
            elif cmd == Command.CancelGoal:
                self.cancelGoal()
            elif cmd == Command.SetGoalBin:
                self.SetBinHelper(int(input("Bin id: ")))
            elif cmd == Command.SetRobotState:
                for state in ROBOT_STATE:
                    print(f"{state.value}: {state.name}")
                self.SetState(ROBOT_STATE(int(input("Robot_State: "))))
            elif cmd == Command.SetPickupState:
                for state in PICKUP_STATE:
                    print(f"{state.value}: {state.name}")
                self.pickup_state = PICKUP_STATE(int(input("Pickup_State: ")))
            elif cmd == Command.GoToGoalBin:
                if self.goal_bin is not None:
                    self.movebase_client(*self.goal_bin.getXY())
                    self.SetState(ROBOT_STATE.FETCHING)
                    self.Loop()
                else:
                    print("No goal bin")
            elif cmd == Command.GoToTakeOut:
                if self.goal_bin is not None:
                    self.movebase_client(*self.goal_bin.getTakeOutPos())
                    self.goal_bin = None
                    self.SetState(ROBOT_STATE.RETURNING)
                    self.Loop()
                else:
                    print("No goal bin")
            elif cmd == Command.GoToHome:
                if self.goal_bin is not None:
                    self.movebase_client(*self.goal_bin.getHomePos())
                    self.goal_bin = None
                    self.SetState(ROBOT_STATE.RETURNING)
                    self.Loop()
                else:
                    print("No goal bin")
            elif cmd == Command.EnterLoop:
                self.Loop()

if __name__ == "__main__":
    try:

        rospy.init_node("goal_publisher")
        rbt = Robot()
        cmd_line = sys.argv[1] if len(sys.argv) > 1 else None

        if cmd_line is not None and cmd_line == "t":
            rbt.HandleManual()

        # TODO: remove for testing 
        # rbt.goal_bin_id = 0
        # rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
        rbt.Loop()
    
    except KeyboardInterrupt:
        ...
