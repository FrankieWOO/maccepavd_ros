#! /usr/bin/env python

import rospy, message_filters
from maccepavd_model import MaccepavdModel
from maccepavd.msg import SensorsRaw, Sensors, CommandRaw, Command, StateCommand
from maccepavd.srv import CallSensors, CallSensorsRequest, CallSensorsResponse, CallRawSensors, CallRawSensorsRequest, CallRawSensorsResponse


def sub_sensors_cb(msg):
    state_cmd = StateCommand()
    state_cmd.header = msg.header
    state_cmd.joint_angle = msg.joint_angle
    state_cmd.servo1_position = msg.servo1_position
    state_cmd.servo2_position = msg.servo2_position
    state_cmd.motor_current = msg.motor_current
    state_cmd.servo1_current = msg.servo1_current
    state_cmd.servo2_current = msg.servo2_current
    cmd = cmd_buffer[-1]
    state_cmd.u1 = cmd.u1
    state_cmd.u2 = cmd.u2
    state_cmd.u3 = cmd.u3
    pub_statecmd.publish(state_cmd)


def sub_rawcmd_cb(msg):
    cmd = Command()
    cmd.u1 = model.servo1_usec2rad(msg.u1)
    cmd.u2 = model.servo2_usec2rad(msg.u2)
    cmd.u3 = msg.u3/255
    cmd_buffer.append(cmd)
    #rawcmd_buffer.append(msg)
    if len(cmd_buffer) > 50:
        del cmd_buffer[0]
        #del rawcmd_buffer[0]


if __name__ == '__main__':
    #sensors_buffer = []
    #rawsensors_buffer = []
    #rawcmd_buffer = []
    cmd_buffer = []
    cmd0 = Command()
    cmd0.u1 = 0
    cmd0.u2 = 0
    cmd0.u3 = 0
    cmd_buffer.append(cmd0)
    model = MaccepavdModel()
    rospy.init_node('statecmd_synchronizer')
    sub_ssr = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    #sub_rawcmd = message_filters.Subscriber('command_raw', CommandRaw, )
    sub_rawcmd = rospy.Subscriber('command_raw', CommandRaw, sub_rawcmd_cb)
    pub_statecmd = rospy.Publisher('state_command', StateCommand, queue_size = 10)
    #pub_sensor = rospy.Publisher('sensors', Sensors, queue_size = 10)
    rospy.spin()
