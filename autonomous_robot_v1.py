# -*- coding: utf-8 -*-
"""
Created on Wed Aug 25 17:29:15 2021

@author: bsmig
"""


import dc_motor
import ultrasonic_sensor

front_left_motor                  = dc_motor(FRONT_LEFT_EN, FRONT_LEFT_IN1, FRONT_LEFT_IN2)
front_right_motor                 = dc_motor(FRONT_RIGHT_EN, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2)
back_left_motor                   = dc_motor(BACK_LEFT_EN, BACK_LEFT_IN1, BACK_LEFT_IN2)
back_right_motor                  = dc_motor(BACK_RIGHT_EN, BACK_RIGHT_IN1, BACK_RIGHT_IN2)

usensor                           = ultrasonic_sensor(TRIGGER_PIN, ECHO_PIN)

while True:
    # make a distance measurement
    current_range = usensor.measure_range_1()
    if (current_range >= 0.5):
        front_left_motor.forward()
        front_right_motor.forward()
        back_left_motor.forward()
        back_right_motor.forward()
    else:
        front_left_motor.stop()
        front_right_motor.stop()
        back_left_motor.stop()
        back_right_motor.stop()