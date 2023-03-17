# Testing drive code with the pyCreate2 library

from pycreate2 import Create2
import time
import keyboard

port = "/dev/tty.usbserial-DN04GQAN"
bot = Create2(port, 19200) # (port, baudrate)

# Start the Create2
bot.start()

# Put the bot into safe mode for driving
bot.safe()

SPEEDL = 200
SPEEDR = 200

# Main drive loop
try:
    while True:
        if (keyboard.is_pressed('z')):
            bot.drive_stop()
            break

        if (keyboard.is_pressed('space')):
            bot.drive_stop()

        elif (keyboard.is_pressed('UP')):
            if (keyboard.is_pressed('RIGHT')):
                bot.drive_direct(0, SPEEDR)
            elif (keyboard.is_pressed('LEFT')):
                bot.drive_direct(SPEEL, 0)
            else:
                bot.drive_direct(SPEEDL, SPEEDR)
        
        elif (keyboard.is_pressed('DOWN')):
            if (keyboard.is_pressed('RIGHT')):
                bot.drive_direct(-SPEEDL, 0)
            elif (keyboard.is_pressed('LEFT')):
                bot.drive_direct(0, -SPEEDR)
            else:
                bot.drive_direct(-SPEEDL, -SPEEDR)
        
        elif (keyboard.is_pressed('RIGHT')):
            bot.drive_direct(-SPEEDL, SPEEDR)
        elif (keyboard.is_pressed('LEFT')):
            bot.drive_direct(SPEEDL, -SPEEDR)


        elif (keyboard.is_pressed('=')):
            if (SPEEDL < 500):
                SPEEDL += 10
                SPEEDR += 10
        elif (keyboard.is_pressed('-')):
            if (SPEEDR > 0):
                SPEEDL -= 10
                SPEEDR -= 10

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nShutting down...\n")
finally:
    try:
        bot.close()
        exit(0)
    except:
        print("Serial close failed")
        exit(0)
