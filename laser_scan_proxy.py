#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

PI = 3.1415

def callback(msg):
    # Modify the laserscan message to have a smaller field of view
    new_msg = LaserScan()
    new_msg.header = msg.header
    new_msg.angle_max = PI/2 
    new_msg.angle_min =  -PI/2
    new_msg.angle_increment = msg.angle_increment
    new_msg.time_increment = msg.time_increment
    new_msg.scan_time = msg.scan_time
    new_msg.range_min = msg.range_min
    new_msg.range_max = msg.range_max
    print(len(msg.ranges)) 
    new_msg.ranges = msg.ranges[270:450]

    # Publish the modified laserscan message on the new topic
    pub.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('laser_scan_proxy')
    pub = rospy.Publisher('/scan_180', LaserScan, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()