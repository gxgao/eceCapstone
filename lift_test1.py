#!/usr/bin/env python

# Copyright (c) 2019-2022, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.


import RPi.GPIO as GPIO
import time

# Pin Definitions
PUL_PIN = 32
DIR_PIN = 31
ENA_PIN = 29

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin numbering scheme
    
    # Set pin as an output pin with optional initial state of HIGH
    GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENA_PIN, GPIO.OUT, initial=GPIO.HIGH)
    
    # p = GPIO.PWM(PUL_PIN, 1000)

    PERIOD = 0.001
    print("Starting demo, press CTRL+C to exit\n")


    curr_value = GPIO.HIGH
    try:
        while True:
            input("Press ENTER to raise cam")
            
            GPIO.output(DIR_PIN, GPIO.LOW)
            GPIO.output(ENA_PIN, GPIO.LOW)
            
            # p.start(50)
            # time.sleep(2.4)
            # p.stop()
            for i in range(2400):
                GPIO.output(PUL_PIN, GPIO.HIGH)
                time.sleep(PERIOD)
                GPIO.output(PUL_PIN, GPIO.LOW)
                time.sleep(PERIOD)

            
            GPIO.output(ENA_PIN, GPIO.HIGH)
            input("Press ENTER to lower cam")
            GPIO.output(DIR_PIN, GPIO.HIGH)
            GPIO.output(ENA_PIN, GPIO.LOW)
            
            # p.start(50)
            # time.sleep(2.4)
            # p.stop()
            for j in range(2400):
                GPIO.output(PUL_PIN, GPIO.HIGH)
                time.sleep(PERIOD)
                GPIO.output(PUL_PIN, GPIO.LOW)
                time.sleep(PERIOD)

            GPIO.output(ENA_PIN, GPIO.HIGH)
            GPIO.output(DIR_PIN, GPIO.LOW)
            print()

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
