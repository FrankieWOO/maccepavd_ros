#! /usr/bin/env python

import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))

print(sys.version_info.major)
if sys.version_info.major > 2:
    import configparser
    config = configparser.ConfigParser()
else:
    import ConfigParser
    config = ConfigParser.ConfigParser()


import rospy, math
#import ConfigParser
from maccepavd.msg import CommandRaw, Command, Sensors, SensorsRaw

class MaccepavdModel(object):
    u1_usec_min = 900
    u1_usec_max = 2100
    u1_deg_min = -60
    u1_deg_max = 60
    u1_rad_min = math.radians(u1_deg_min)
    u1_rad_max = math.radians(u1_deg_max)
    u2_usec_min = 900
    u2_usec_max = 2100
    u2_deg_min = 0
    u2_deg_max = 120
    u2_rad_min = math.radians(u2_deg_min)
    u2_rad_max = math.radians(u2_deg_max)
    gear_ratio = 1

    """ weights and offsets values for transfering sensor readings (in voltage)
    # to radians. moved to ini setting file
    w0s2r = 1.224148
    w1s2r = 1.261684
    w2s2r = 1.265037
    off0s2r = -3.17018
    off1s2r = -1.92962
    off2s2r = -0.87958
    """
    off_dmc = 2.56142492771
    off_s1c = 2.44158988476
    off_s2c = 2.53156570911


    def __init__(self):
        """
        read config file settings_sensors.ini and
        """
        self.u1_usec_range = self.u1_usec_max - self.u1_usec_min
        self.u1_deg_range = self.u1_deg_max - self.u1_deg_min
        self.u1_rad_range = self.u1_rad_max - self.u1_rad_min
        self.u1_usec_mid = self.u1_usec_range/2 + self.u1_usec_min
        self.u2_usec_range = self.u2_usec_max - self.u2_usec_min
        self.u2_deg_range = self.u2_deg_max - self.u2_deg_min
        self.u2_rad_range = self.u2_rad_max - self.u2_rad_min
        self.w1r2u = self.u1_usec_range/self.u1_rad_range
        self.w2r2u = self.u2_usec_range/self.u2_rad_range
        config.read(dir_path+'/settings_sensors.ini')
        toread = 'adc'
        # use raw ADC sensor data, setup adc-rad conversion
        if toread == 'adc':
            self.w0s2r = float(config.get('joint-adc2rad','weight'))
            self.off0s2r = float(config.get('joint-adc2rad','offset'))
            self.w1s2r = float(config.get('servo1-adc2rad','weight'))
            self.off1s2r = float(config.get('servo1-adc2rad','offset'))
            self.w2s2r = float(config.get('servo2-adc2rad','weight'))
            self.off2s2r = float(config.get('servo2-adc2rad','offset'))
        else:
            self.w0s2r = float(config.get('joint-vol2rad','weight'))
            self.off0s2r = float(config.get('joint-vol2rad','offset'))
            self.w1s2r = float(config.get('servo1-vol2rad','weight'))
            self.off1s2r = float(config.get('servo1-vol2rad','offset'))
            self.w2s2r = float(config.get('servo2-vol2rad','weight'))
            self.off2s2r = float(config.get('servo2-vol2rad','offset'))


    def cmd2raw(self, cmd):
        rawcmd = CommandRaw()
        #rawcmd.u1 = self.servo1_rad2usec(cmd.u1)
        #rawcmd.u2 = self.servo2_rad2usec(cmd.u2)
        rawcmd.u1 = round(math.degrees(cmd.u1)/0.087890625)+2048
	rawcmd.u2 = round(math.degrees(cmd.u2)/0.087890625)+512
        if cmd.u3 <= 0.5:
            D1 = cmd.u3/0.5
            D2 = 0
        else:
            D1 = 1
            D2 = (cmd.u3 - 0.5)/0.5

        rawcmd.D1 = self.dc2adc(D1)
        rawcmd.D2 = self.dc2adc(D2)
        return rawcmd

    def raw2sensors(self, rawsensor_msg):
        # convert raw sensors reading to meaningful values
        sensor_msg = Sensors()
        sensor_msg.header = rawsensor_msg.header
        sensor_msg.joint_position = self.joint_sensor2rad(rawsensor_msg.joint_sensor)
        sensor_msg.rege_current = rawsensor_msg.rege_current
        return sensor_msg

    def servo1_rad2usec(self, rad):
        usec = rad * self.w1r2u + self.u1_usec_mid
        usec = round(usec)
        return usec

    def servo1_usec2rad(self, usec):
        rad = (usec - self.u1_usec_mid)/self.w1r2u
        return rad

    def servo1_deg2usec(self, deg):
        usec = (deg + 60) * self.u1_usec_range / self.u1_deg_range + self.u1_usec_min
        usec = round(usec)
        return usec

    def servo1_sensor2rad(self,read):
        # convert raw sensor read to radium
        rad = self.w1s2r*read + self.off1s2r
        return rad

    def servo2_rad2usec(self, rad):
        usec = rad * self.w2r2u + self.u2_usec_min
        usec = round(usec)
        return usec

    def servo2_usec2rad(self, usec):
        rad = (usec - self.u2_usec_min)/self.w2r2u
        return rad

    def servo2_deg2usec(self, deg):
        usec = deg * self.u2_usec_range / self.u2_deg_range + self.u2_usec_min
        usec = round(usec)
        return usec

    def servo2_sensor2rad(self,read):
        # convert raw sensor read to radium
        #
        rad = self.w2s2r*read + self.off2s2r
        return rad

    def joint_sensor2rad(self, read):
        rad = self.w0s2r*read + self.off0s2r
        return rad

    def dc2adc(self, dc):
        # convert duty circle to analog input
        # dc := [0,1]
        adc = round(dc * 255)
        return adc

    def adc2dc(self, adc):
        dc = adc/255
        return dc
