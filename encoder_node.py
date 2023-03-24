#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int16
import rospy
from std_msgs.msg import Int16


def rightEncoderPub():
    pub = rospy.Publisher('RightEncoder', Int16, queue_size=10)
    rospy.init_node('leftEncoderPublisher', anonymous=True)
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            pub.publish(bot.get_encoder_counts()['right'])
    #        lpub.publish(1)

            rate.sleep()
    #       lpub.sleep()
        except rospy.ROSInterruptException:
            pass
 

def LeftEncoderPub():
    # topic is right encoder 
    pub = rospy.Publisher('rightEncoder', Int16, queue_size=10)
    # node name is right encoder publisher 
    rospy.init_node('rightEncoderPublisher', anonymous=True)
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            pub.publish(bot.LeftEncoderValue  )
    #        lpub.publish(1)

            rate.sleep()
    #       lpub.sleep()
        except rospy.ROSInterruptException:
            pass
 

if __name__ == '__main__':
    
    rpub = rightEncoderPub() 
    #lpub = LeftEncoderPub() 
    rate = rospy.Rate(10) #10hz
    
    while not rospy.is_shutdown():
        try:
            rpub.publish(2)
    #        lpub.publish(1)

            rate.sleep()

     #       lpub.sleep()
        except rospy.ROSInterruptException:
            pass
        
    
