#! /usr/bin/env python

import glob
import numpy as np
import os, sys
import scipy.io, numpy, rospy, actionlib
import pandas as pd
import matplotlib.pyplot as plt
import subprocess
from maccepavd.msg import CommandRaw, SensorsRawAdcCmd
#from maccepavd_model import MaccepavdModel
from record_ros.srv import String_cmd
from maccepavd.srv import StartRecord, StopRecord
from datetime import datetime


def test():
    u1 = 1500
    u2 = 900
    D1 = 0
    D2 = 0
    pub_rawcmd.publish(u1, u2, D1, D2)
    rospy.sleep(1.)
    u1 = 2000
    pub_rawcmd.publish(u1, u2, D1, D2)
    t0 = rospy.get_rostime()
    t0 = t0.to_sec()
    global record
    record = True
    t = 0
    while t<0.02:
        t = rospy.get_rostime()
        t = t.to_sec() - t0

    record = False
    global time
    time = np.array(time) - t0
    global u
    u = np.array(u)
    print(time)
    print(len(time))
    print(len(u))
    plt.plot(time, u)
    plt.show()




def plot_tj(u, xsim, tsim, xm, tm, xservo1):
    fig, axs = plt.subplots(3,1, sharex = True)
    axs[0].plot(tsim,xsim[0,:],tm,xm )
    axs[1].plot(tsim[0:-1], u[0,:], tsim[0:-1], u[1,:],tm, xservo1)
    axs[2].plot(tsim[0:-1],u[2,:])
    plt.show()
    #legend()

def sub_sensors_cb(msg):
    if record:
        time.append(msg.header.stamp.to_sec())
        u.append(msg.u1)


if __name__ == '__main__':
    rospy.init_node('test_latency')
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)
    sub_rawssr = rospy.Subscriber('sensors_raw', SensorsRawAdcCmd, sub_sensors_cb)
    record = False
    time = []
    u = []
    test()
