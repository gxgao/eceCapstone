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

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

X, Y, Z = 0, 1, 2
# central axis of robot
FRONT_CAMERA_CENTRAL = -9
PI = 3.1415

@dataclass
class Vector:
    z: int
    l_x: int
    r_x: int
    speed: float
    dist: float

# TODO: add array for slim side
back_camera_array = [
    Vector(105, -48, -31, -.1, .1),
    Vector(85, -35, -27, -.1, .1),
    Vector(67, -31, -25, -.06, .04),
    Vector(56, -26, -24, -.05, .04)
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

    def SetPosition(self, x, y, yaw):
        rpyPose = self.CreatePoseStamped(x, y, yaw)
        rpyPoseWithCovarianceStamped = PoseWithCovarianceStamped()
        rpyPoseWithCovarianceStamped.header.frame_id = rpyPose.header.frame_id
        rpyPoseWithCovarianceStamped.header.stamp = rpyPose.header.stamp
        rpyPoseWithCovarianceStamped.pose.pose.position = rpyPose.pose.position
        rpyPoseWithCovarianceStamped.pose.pose.orientation = rpyPose.pose.orientation

        print("init_2d_pub")
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

        while abs(rbt.rot - goal_angle) > 1.0:
            if (goal_angle >= 0 and rbt.rot >= 0) or (goal_angle <= 0 and rbt.rot <= 0):
                clockwise = goal_angle >= rbt.rot
            elif rbt.rot <= 0 and goal_angle >= 0:
                clockwise = goal_angle - rbt.rot <= (180 + rbt.rot) + 180 - goal_angle
            elif rbt.rot >= 0 and goal_angle <= 0:
                clockwise = (180 - rbt.rot) + (180 + goal_angle) <= rbt.rot - goal_angle
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
        rbt.pos = position[:2]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        rbt.rot = yaw * 180 / 3.1415926

    def HandleIdle(self):
        freeBin = rbt.binTracker.getFreeBin()
        x, y = freeBin.getXY()
        print(f"free bin {float(x)}, {float(y)}")
        rbt.state = ROBOT_STATE.FETCHING
        # rbt.sendGoal(float(x), float(y), float(0.0))
        self.movebase_client(x, y)
        self.goal_bin_id = freeBin.binId

    def HandleFetching(self):
        if self.cnt % 10 == 0:
            distAndMarkers = va.fetch_bin_distance_vec()
            print(distAndMarkers)
            if distAndMarkers is not None:
                dist, marker = distAndMarkers
                print("saw marker")
                # if within 110 cm we will swap our state into docking mode
                if (
                    marker[0] == rbt.binTracker.binToGet.arucoId
                    and dist[Z] < 150
                ):
                    # do some movement towards it
                    print("Cancel Goal with dist z:", dist[Z])
                    self.client.cancel_goal()
                    self.state = ROBOT_STATE.DOCKING
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            print("Reached destination")
            self.state = ROBOT_STATE.SEARCHING_FOR_BIN
    
    def DetectBin(self, forward=True): 
        for _ in range(5):
            distAndMarkers = va.fetch_bin_distance_vec(forward) 
            if (
                distAndMarkers is not None
                and self.binTracker.ArucoIdToBinId(distAndMarkers[1][0][0])  == self.goal_bin_id
            ):
                return distAndMarkers 
        return None 
        
    def DetectBinOrientation(self, forward=True):
        distAndMarkers = self.DetectBin()
        if distAndMarkers is not None:
            _, markers = distAndMarkers
            return self.binTracker.GetBinSide(markers[0][0])
        return None

    def HandleSearchingForBin(self):
        if (self.DetectBin(forward=False)):
            self.state = ROBOT_STATE.DOCKING
            return 
    
        distAndMarkers = self.DetectBin()

        if distAndMarkers is not None:
            dist, marker = distAndMarkers
            print(
                "marker detected while searching using front camera", marker
            )
            print(self.goal_bin_id)
            # x axis is horizontal to robot, right is decreasing
            # moving roomba up increases y
            # z is obvious:

            x, _, z = dist
            print("z is ", z)
            if z <= 30:
                self.drive_bot(-0.1, 0.1)
            elif z >= 100:
                self.drive_bot(0.1, 0.1)
            elif x < -40:
                self.rotate_new(8, self.rot + 4)
            elif x > -20:
                self.rotate_new(8, self.rot - 4)
            else:
                print("here")
                self.rotate_new(15, self.rot + 180)
                self.state = ROBOT_STATE.DOCKING
        else:
            # rotate in place until you find it, if rotated 2x we will go back to goal movement
            if self.cnt % 10 == 0:
                self.rotate(8, 25)

    def ShouldLoop(self):
        return self.running and not rospy.is_shutdown()

    def HandleDocking(self):
        print("docking")
        distAndMarkers = self.DetectBin(forward=False) 

        if (distAndMarkers is None):
            print("swapping back to searching")
            rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
            return 

        dist, markers = distAndMarkers
        print("marker detected", markers)
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
                if i == len(back_camera_array) - 1:
                    sleep_extra = True
                break
        print(f"picked {calibrate} and sleep_extra={sleep_extra}")
        
        
        def lerp(x1, y1, x2, y2, x):
            return y1 + (y2 - y1) / (x2 - x1) * (x - x1)

        if calibrate is not None:
            # if not last range, we look at prev indx and do linear 
            # interpolation to find what range we should be within 
            if i > 0:
                prev_calibrate = back_camera_array[i - 1]
            else: 
                prev_calibrate = back_camera_array[0]
                calibrate = back_camera_array[1]

            l_x = lerp(prev_calibrate.z, prev_calibrate.l_x, calibrate.z, calibrate.l_x, z) 
            r_x = lerp(prev_calibrate.z, prev_calibrate.r_x, calibrate.z, calibrate.r_x, z) 

            print("my x", x)
            if x < l_x:
                print(f"{x} < {l_x}")
                self.rotate_new(5.5, self.rot - 1.5, sleep_extra)
            elif x > r_x:
                print(f"{x} > {r_x}")
                self.rotate_new(5.5, self.rot + 1.5, sleep_extra)
            else:
                print(f"{l_x} < {x} < {r_x}")
                self.drive_bot(
                    calibrate.speed, calibrate.dist
                )  # i hope since we sleep min_dist gets edited
            if sleep_extra:
                self.ForceSleepSec(0.25)
        else:
            input("now going straight in")

            def get_min_dist():
                rear_laser_ranges = self.laser_ranges[500:650]
                min_dist = min(rear_laser_ranges)
                return min_dist

            side = self.binTracker.GetBinSide(markers[0][0])
            min_dist_limit = .27 if side == binID.BIN_SIDE.WIDE else .22

            distances = []
            while get_min_dist() > min_dist_limit:
                min_dist = get_min_dist()
                distances.append(min_dist)
                print("min_dist ", min_dist)
                dist_to_travel = min(0.03, max(.02, min_dist - min_dist_limit))
                print("dist to travel ", dist_to_travel)
                # self.drive_back(0.05, dist_to_travel)
                self.drive_bot(-.05, dist_to_travel)
                self.ForceSleepSec(0.25)

                if len(distances) > 3 and abs(distances[-1] - distances[-3]) < .01:
                    break
                prev_min_dist = min_dist

            # add lift code ...
            # self.arms.raise_arms()

            # post lift code
            # self.movebase_client(*self.get_dropoff())
            # self.state = ROBOT_STATE.RETURNING
            self.drive_bot(-.2, .08)
            print("done")
            self.running = False
    
    def HandleReturning(self):
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.state = ROBOT_STATE.DEDOCKING
    
    def HandleDedocking(self):
        # self.arms.lower_arms()

        rob_x, rob_y = self.pos
        arm_offset = 0.2794
        ang = self.rot
        arm_ang = ang + 180 % 360
        bin_x = arm_offset * cos(arm_ang) + rob_x
        bin_y = arm_offset * sin(arm_ang) + rob_y
        self.binTracker.binDrop(True, bin_x, bin_y)

        self.state = ROBOT_STATE.IDLE
        self.drive_bot(-.05, .5)

    def RunLoopOnce(self):
        print(self.state)
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
            cmd = input("goal (g) drive (d) rotate (r)?")
            if cmd == "g":
                action = input("x y yaw").split()
                if len(action) <= 1:
                    self.client.cancel_goal()
                else:
                    x, y, yaw = action
                    # rbt.sendGoal(float(x), float(y), float(yaw))
                    self.movebase_client(x, y)
            elif cmd == "d":
                action = input("speed x, x").split()
                speedx, x = action
                self.drive_bot(speedx, x)
            elif cmd == "da":
                action = input("speed x, x, a").split()
                speedx, x, a = action
                self.drive_bot(speedx, x, a)
            elif cmd == "db":
                action = input("speed x, x").split()
                speedx, x = action
                speedx = float(speedx)
                x = float(x)
                self.drive_back(speedx, x, splits=10)
            elif cmd == "r":
                angle = input("angle")
                self.rotate(30, float(angle))
            elif cmd == "sp":
                x, y, yaw = input("x, y, yaw").split()
                self.SetPosition(float(x), float(y), float(yaw))
            elif cmd == "b":
                while True:
                    self.movebase_client(3.0, 0.0)
                    time.sleep(45)
                    self.movebase_client(0.0, 0.0)
                    time.sleep(45)
            elif cmd == "ra":
                self.arms.raise_arms()
            elif cmd == "la":
                self.arms.lower_arms()
            elif cmd == "cg":
                self.cancelGoal()

if __name__ == "__main__":
    try:

        rospy.init_node("goal_publisher")
        rbt = Robot()
        cmd_line = sys.argv[1] if len(sys.argv) > 1 else None

        if cmd_line is not None and cmd_line == "t":
            rbt.HandleManual()

        # TODO: remove for testing 
        # rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
        rbt.goal_bin_id = 0
        rbt.state = ROBOT_STATE.SEARCHING_FOR_BIN
        rbt.Loop()
    
    except KeyboardInterrupt:
        ...
