# -*- coding: utf-8 -*-
"""
Created on Wed Aug 25 06:17:54 2021

@author: bsmig
"""

import RPi.GPIO as GPIO
import time
from parameters import *

class ultrasonic_sensor:
    def __init__(self,
                 TRIGGER,
                 ECHO): 
        
        self.TRIGGER       = TRIGGER
        self.ECHO          = ECHO
        
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self.TRIGGER, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        
    def measure_range_1(self):
        # make sure pin is low before beginning
        GPIO.output(self.TRIGGER, GPIO.LOW)
        time.sleep(0.1)
        
        # the sensor sends out a 10 us pulse
        # so set the trigger to high, wait 10 us
        # and then set low again
        GPIO.output(self.TRIGGER, GPIO.HIGH)
        time.sleep(0.00001) # 10 us = 1e-5 s
        GPIO.output(self.TRIGGER, GPIO.LOW)
        
        # get the last time the pin is low:
        # meaning right before the echo pulse's
        # rising edge hits
        while (GPIO.input(self.ECHO) == 0):
            pulse_start = time.time()
            
        # get the last time the pin is high:
        # meaning right before the echo pulse's
        # falling edge passes
        while (GPIO.input(self.ECHO) == 1):
            pulse_stop = time.time()
            
        # what confuses me is this is not how a radar
        # works. In a radar the actual pulse width
        # remains unchanged and it truly is the time
        # between the emitted and received pulse
        # that tells you about the range to the object.
        # What I have learned is that this sensor
        # still does this but encodes that information
        # into a "signal" or control pulse present on the echo pin
        # whose pulse width is proportional to the range:
            
        echo_pulse_duration = pulse_stop - pulse_start
        range_to_object_m = 0.5 * (echo_pulse_duration * 343)
        
        return range_to_object_m
            
    def measure_range_2(self):
        # DO THIS BY SETTING INTERRUPTS FOR 
        # RECEIVED ECHO'S RISING AND FALLING EDGE
        
    def object_detection(self):
        while reading:
            range_to_object_m = self.measure_distance_1()
            if (range_to_object_m <= CLOSEST_RANGE_M):
                # THROW SOME KIND OF FLAG SO IT CAN STOP OR BACKUP
            
            time.sleep(0.01)
            
        