#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int16

def rightEncoderPub():
    pub = rospy.Publisher('leftEncoder', Int16, queue_size=10)
    rospy.init_node('leftEncoderPublisher', anonymous=True)
    return pub

def LeftEncoderPub():
    # topic is right encoder 
    pub = rospy.Publisher('rightEncoder', Int16, queue_size=10)
    # node name is right encoder publisher 
    rospy.init_node('rightEncoderPublisher', anonymous=True)
    return pub


if __name__ == '__main__':
    
    rpub = rightEncoderPub() 
    lpub = LeftEncoderPub() 
    rate = rospy.Rate(10) #10hz
    
    while not rospy.is_shutdown():
        try:
            rpub.publish(2)
            lpub.publish(1)

            rpub.sleep()

            lpub.sleep()
        except rospy.ROSInterruptException:
            pass
        
    