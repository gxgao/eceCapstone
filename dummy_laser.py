import rospy
from geometry_msgs.msg import PoseStamped 
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

def callback(data):
    print(min(data.ranges))


rospy.init_node('goal_publisher')
rate = rospy.Rate(10)
sub = rospy.Subscriber('/scan', LaserScan, callback)


while 1:
    rate.sleep() 
  
