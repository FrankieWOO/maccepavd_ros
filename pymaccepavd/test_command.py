#! /usr/bin/env python

import rospy
from maccepavd.msg import CommandRaw
from maccepavd_model import MaccepavdModel


def send_rawcmd(u1, u2, u3):
    cmd_msg = CommandRaw()
    cmd_msg.u1 = u1
    cmd_msg.u2 = u2
    cmd_msg.u3 = u3
    pub_rawcmd.publish(cmd_msg)


def test_send_rawcmd_seq():
    u1 = [1000, 1100, 1200, 1300, 1500]
    u2 = [1000, 1100, 1200, 1300, 1500]
    u3 = [100, 110, 120, 130, 150]
    i = 0
    loop_rate = rospy.Rate(50)
    while i < 5:
        pub_rawcmd.publish(u1[i], u2[i], u3[i])
        i = i + 1
        loop_rate.sleep()


def send_cmd(u1, u2, u3):
    u1 = model.servo1_rad2usec(u1)
    u2 = model.servo2_rad2usec(u2)
    u3 = model.dc2adc(u3)
    cmd_msg = CommandRaw()
    cmd_msg.u1 = u1
    cmd_msg.u2 = u2
    cmd_msg.u3 = u3
    pub_rawcmd.publish(cmd_msg)


def test_send_cmd_seq():
    u1 = [1000, 1100, 1200, 1300, 1500]
    u2 = [1000, 1100, 1200, 1300, 1500]
    u3 = [100, 110, 120, 130, 150]
    i = 0
    loop_rate = rospy.Rate(50)
    while i < 5:
        pub_rawcmd.publish()
        i = i + 1
        loop_rate.sleep()


rospy.init_node('test_commandraw')
u1 = 1500
u2 = 1500
D1 = 0
D2 = 0
pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)
pub_rawcmd.publish(u1, u2, D1, D2)
model = MaccepavdModel()
