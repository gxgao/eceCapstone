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
DIR_PIN = 12
ENA_PIN = 11
ENA_PIN_2 = 29

ENA_ON = GPIO.LOW
ENA_OFF = GPIO.HIGH

ENA_ON_2 = GPIO.LOW
ENA_OFF_2 = GPIO.HIGH

PERIOD = 0.001
OLD_ITERS = 2600

OLD_RAISE_ITERS = 2800
RAISE_ITERS = 3400
LOWER_ITERS = 2600

import atexit
GPIO.setmode(GPIO.BOARD)  # BOARD pin numbering scheme
GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA_PIN, GPIO.OUT, initial=ENA_OFF)
GPIO.setup(ENA_PIN_2, GPIO.OUT, initial=ENA_OFF_2)

GPIO.output(ENA_PIN, ENA_OFF)
GPIO.output(ENA_PIN_2, ENA_OFF_2)
GPIO.output(DIR_PIN, GPIO.LOW)

atexit.register(GPIO.cleanup)

class Arms:

    def disable_motors(self):
        GPIO.output(ENA_PIN, ENA_OFF)
        GPIO.output(ENA_PIN_2, ENA_OFF_2)
        GPIO.output(DIR_PIN, GPIO.LOW)

    def enable_motors(self):
        GPIO.output(ENA_PIN, ENA_ON)
        GPIO.output(ENA_PIN_2, ENA_ON_2)
    
    def force_lower(self):
        GPIO.output(DIR_PIN, GPIO.LOW)
        self.run_arms(LOWER_ITERS, PERIOD)
        self.raised = False
    
    def force_raise(self):
        GPIO.output(DIR_PIN, GPIO.HIGH)
        self.run_arms(RAISE_ITERS, PERIOD)
        self.raised = True

    def raise_arms(self):
        if self.raised:
            return
        GPIO.output(DIR_PIN, GPIO.HIGH)
        self.run_arms(RAISE_ITERS, PERIOD)
        self.raised = True

    def run_arms(self, iters, period):
        self.enable_motors()
        for i in range(iters):
            GPIO.output(PUL_PIN, GPIO.LOW)
            time.sleep(period)
            GPIO.output(PUL_PIN, GPIO.HIGH)
            time.sleep(period)
        self.disable_motors()

    def lower_arms(self):
        if not self.raised:
            return
        
        GPIO.output(DIR_PIN, GPIO.LOW)
        self.run_arms(LOWER_ITERS, PERIOD)
        self.raised = False
    
    def __init__(self):
        # Set pin as an output pin with optional initial state of HIGH
        self.disable_motors()
        self.raised = False



def main():
    print("Starting demo, press CTRL+C to exit\n")
    while True:
        time.sleep(10)


if __name__ == '__main__':
    main()
