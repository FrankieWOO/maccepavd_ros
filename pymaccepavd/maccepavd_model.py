#! /usr/bin/env python

import rospy, math
from maccepavd.msg import CommandRaw, Command, Sensors, SensorsRaw

class MaccepavdModel(object):
    u1_usec_min = 900
    u1_usec_max = 2100
    u1_deg_min = -60
    u1_deg_max = 60
    u1_rad_min = math.radians(u1_deg_min)
    u1_rad_max = math.radians(u1_deg_max)
    u2_usec_min = 650
    u2_usec_max = 2300
    u2_deg_min = 0
    u2_deg_max = 180
    u2_rad_min = math.radians(u2_deg_min)
    u2_rad_max = math.radians(u2_deg_max)

    def __init__(self):
        self.u1_usec_range = self.u1_usec_max - self.u1_usec_min
        self.u1_deg_range = self.u1_deg_max - self.u1_deg_min
        self.u2_usec_range = self.u2_usec_max - self.u2_usec_min
        self.u2_deg_range = self.u2_deg_max - self.u2_deg_min

    def cmd2raw(self, cmd):
        rawcmd = CommandRaw()
        rawcmd.u1 = self.servo1_rad2usec(cmd.u1)
        rawcmd.u2 = self.servo2_rad2usec(cmd.u2)
        rawcmd.u3 = self.dc2adc(cmd.u3)
        return rawcmd

    def raw2sensors(self, rawsensor_msg):
        # convert raw sensors reading to meaningful values
        # todo: calibrate joint sensor, joint sensor is ustable now
        sensor_msg = Sensors()
        sensor_msg.header = rawsensor_msg.header
        sensor_msg.servo1_position = self.servo1_sensor2rad(rawsensor_msg.servo1_sensor)
        sensor_msg.servo2_position = self.servo2_sensor2rad(rawsensor_msg.servo2_sensor)
        sensor_msg.joint_angle = rawsensor_msg.joint_sensor
        return sensor_msg

    def servo1_rad2usec(self, rad):
        usec = (math.degrees(rad) + 60) * self.u1_usec_range / self.u1_deg_range + self.u1_usec_min
        usec = round(usec)
        return usec

    def servo1_deg2usec(self, deg):
        usec = (deg + 60) * self.u1_usec_range / self.u1_deg_range + self.u1_usec_min
        usec = round(usec)
        return usec

    def servo1_sensor2rad(self,read):
        # convert raw sensor read to radium
        deg = - (read - 296)*60/166
        rad = math.radians(deg)
        return rad

    def servo2_rad2usec(self, rad):
        usec = math.degrees(rad) * self.u2_usec_range / self.u2_deg_range + self.u2_usec_min
        usec = round(usec)
        return usec

    def servo2_deg2usec(self, deg):
        usec = deg * self.u2_usec_range / self.u2_deg_range + self.u2_usec_min
        usec = round(usec)
        return usec

    def servo2_sensor2rad(self,read):
        # convert raw sensor read to radium
        #
        deg = - (read - 596) * 180 / 527
        rad = math.radians(deg)
        return rad

    def dc2adc(self, dc):
        # convert duty circle to analog input
        # dc := [0,1]
        adc = round(dc * 255)
        return adc

    def adc2dc(self, adc):
        dc = adc/255
        return dc
