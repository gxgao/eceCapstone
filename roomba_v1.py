# Testing drive code with the pyCreate2 library

from pycreate2 import Create2
import time
from sshkeyboard import listen_keyboard
from multiprocessing import Process

import rospy
from std_msgs.msg import Int16


def rightEncoderPub():
    pub = rospy.Publisher('RightEncoder', Int16, queue_size=10)
    rospy.init_node('rightEncoderPublisher', anonymous=True)
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            pub.publish(bot.get_sensors().encoder_counts_right )
    #        lpub.publish(1)

            rate.sleep()
    #       lpub.sleep()
        except rospy.ROSInterruptException:
            pass
 

def LeftEncoderPub():
    # topic is right encoder 
    pub = rospy.Publisher('leftEncoder', Int16, queue_size=10)
    # node name is right encoder publisher 
    rospy.init_node('leftEncoderPublisher', anonymous=True)
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            pub.publish(bot.get_sensors().encoder_counts_left  )
    #        lpub.publish(1)

            rate.sleep()
    #       lpub.sleep()
        except rospy.ROSInterruptException:
            pass


port = "/dev/ttyUSB1"
bot = Create2(port, 19200) # (port, baudrate)

# Start the Create2
bot.start()

procs = []

# print(name)
proc = Process(target=rightEncoderPub)
procs.append(proc)
proc.start()

proc = Process(target=LeftEncoderPub)
procs.append(proc)
proc.start()



# Put the bot into safe mode for driving
bot.safe()

VELOCITYCHANGE = 100
ROTATIONCHANGE = 100

class KeyboardHandler:
    def __init__(self, bot):
        self.callbackKeyUp = False
        self.callbackKeyDown = False
        self.callbackKeyLeft = False
        self.callbackKeyRight = False
        self.bot = bot

    def press(self, key):
        if key == 'up':
            self.callbackKeyUp = True
        elif key == 'down':
            self.callbackKeyDown = True
        elif key == 'left':
            self.callbackKeyLeft = True
        elif key == 'right':
            self.callbackKeyRight = True
        else:
            return
        self.handle_movement()

    def release(self, key):
        if key == 'up':
            self.callbackKeyUp = False
        elif key == 'down':
            self.callbackKeyDown = False
        elif key == 'left':
            self.callbackKeyLeft = False
        elif key == 'right':
            self.callbackKeyRight = False
        else:
            return
        self.handle_movement()

    def handle_movement(self):
        velocity = 0
        velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
        velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
        rotation = 0
        rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
        rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

        # compute left and right wheel velocities
        vr = int(velocity + (rotation/2))
        vl = int(velocity - (rotation/2))
        print(vl, vr)

        self.bot.drive_direct(vr, vl)
bot.drive_direct(100, -100)
time.sleep(2)
bot.drive_stop()
keyboardHandler = KeyboardHandler(bot)

listen_keyboard(
    on_press=keyboardHandler.press,
    on_release=keyboardHandler.release,
    until="space",
)

bot.drive_stop()

# complete the processes
for proc in procs:
    proc.join()

bot.close()


# Main drive loop
#try:
#    while True:
#        if (keyboard.is_pressed('z')):
#            bot.drive_stop()
#            break
#
#        if (keyboard.is_pressed('space')):
#            bot.drive_stop()
#
#        elif (keyboard.is_pressed('UP')):
#            if (keyboard.is_pressed('RIGHT')):
#                bot.drive_direct(0, SPEEDR)
#            elif (keyboard.is_pressed('LEFT')):
#                bot.drive_direct(SPEEL, 0)
#            else:
#                bot.drive_direct(SPEEDL, SPEEDR)
#        
#        elif (keyboard.is_pressed('DOWN')):
#      dafdslfajpressed('RIGHT')):
#                bot.drive_direct(-SPEEDL, 0)
#            elif (keyboard.is_pressed('LEFT')):
#                bot.drive_direct(0, -SPEEDR)
#            else:
#                bot.drive_direct(-SPEEDL, -SPEEDR)
#        
#        elif (keyboard.is_pressed('RIGHT')):
#            bot.drive_direct(-SPEEDL, SPEEDR)
#        elif (keyboard.is_pressed('LEFT')):
#            bot.drive_direct(SPEEDL, -SPEEDR)
#
#
#        elif (keyboard.is_pressed('=')):
#            if (SPEEDL < 500):
#                SPEEDL += 10
#                SPEEDR += 10
#        elif (keyboard.is_pressed('-')):
#            if (SPEEDR > 0):
#                SPEEDL -= 10
#                SPEEDR -= 10
#
#        time.sleep(0.02)
#
#except KeyboardInterrupt:
#    print("\nShutting down...\n")
#finally:
#    try:
#        bot.close()
#        exit(0)
#    except:
#        print("Serial close failed")
#        exit(0)
