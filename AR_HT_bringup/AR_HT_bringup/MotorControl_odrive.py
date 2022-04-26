#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
information for future configuration
odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.controller.config.vel_limit = 58 
odrv0.axis0.motor.config.calibration_current = 5
odrv0.config.brake_resistance = 0
odrv0.axis0.motor.config.pole_pairs = 6
odrv0.axis0.motor.config.torque_constant = 0.045
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.cpr = 1024 * 8
odrv0.axis1.motor.config.current_lim = 10
odrv0.axis1.controller.config.vel_limit = 58 
odrv0.axis1.motor.config.calibration_current = 5
odrv0.config.brake_resistance = 0
odrv0.axis1.motor.config.pole_pairs = 6
odrv0.axis1.motor.config.torque_constant = 0.045
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.encoder.config.cpr = 1024 * 8 
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_offset_calibration = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
odrv0.reboot()
"""

from __future__ import print_function
import odrive
from odrive.enums import *
import time



class MotorControl():
    def __init__(self):
        #init_odrive serial number in
        self.odrv0 = odrive.find_any(serial_number = "207030735432")
        print(self.odrv0)
    def goal_velocity(self, motorId, rps):
        if motorId == 'r': 
            self.odrv0.axis0.controller.input_vel = rps
        if motorId == 'l': 
            self.odrv0.axis1.controller.input_vel = rps
		# print(rps)
    def get_speed(self, motorId):
        if motorId == 'r': 
            return self.odrv0.axis0.encoder.vel_estimate
        if motorId == 'l': 
            return self.odrv0.axis1.encoder.vel_estimate
    def get_voltage(self):
        return self.odrv0.vbus_voltage  
if __name__ == '__main__':
    motor = MotorControl()
    # motor.goal_velocity('', 4)
    time.sleep(1)
    # print('speed = ', motor.set_speed(0))
    motor.goal_velocity(0, 0)
    motor.goal_velocity(1, 0)
    motor.goal_velocity(2, 0)
    motor.goal_velocity(3, 0)