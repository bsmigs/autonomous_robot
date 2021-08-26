# -*- coding: utf-8 -*-
"""
Created on Sat Aug 14 12:57:59 2021

@author: bsmig
"""

import RPi.GPIO as GPIO
from parameters import *

class dc_motor:
    def __init__(self,
                 en
                 in1,
                 in2): 
        
        self.en                = en
        self.in1               = in1
        self.in2               = in2
        
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        
        GPIO.output(self.en, GPIO.LOW)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        
        self.p                 = GPIO.PWM(self.en, PULSE_FREQ_HZ)
        self.p.start(DUTY_CYCLE)
        
    def shutdown(self):
        self.p.stop()
        GPIO.cleanup()
        
    def forward(self, 
                duty_cycle=25):
        self.p.ChangeDutyCycle(duty_cycle)
        
        GPIO.output(self.en, GPIO.HIGH)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        
    def backward(self,
                 duty_cycle=25):
        self.p.ChangeDutyCycle(duty_cycle)
        
        GPIO.output(self.en, GPIO.HIGH)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        
    def stop(self):
        GPIO.output(self.en, GPIO.LOW)