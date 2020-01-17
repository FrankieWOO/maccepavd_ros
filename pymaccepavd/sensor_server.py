#! /usr/bin/env python

import rospy, message_filters
from maccepavd_model import MaccepavdModel
from maccepavd.msg import SensorsRawAdc, Sensors, CommandRaw, Command, StateCommand
from maccepavd.srv import CallSensors, CallSensorsRequest, CallSensorsResponse, CallRawSensors, CallRawSensorsRequest, CallRawSensorsResponse
from sensor_msgs.msg import JointState


def call_sensor_cb(request):
    # todo: return n sensors msg
    # n = request.n
    return CallSensorsResponse(sensors_buffer[-1])


def call_rawsensor_cb(request):
    return CallRawSensorsResponse(rawsensors_buffer[-1])


def sub_sensors_cb(msg, jntstate):
    # convert and store the sensor reading into buffer
    #sensor_msg = Sensors()
    #sensor_msg = model.raw2sensors(msg)
    sensor_msg = Sensors()
    sensor_msg.header = msg.header
    #sensor_msg.header.stamp = rospy.Time.now()
    sensor_msg.joint_position = model.joint_sensor2rad(msg.joint_sensor)
    sensor_msg.rege_current = msg.rege_current
    sensor_msg.servo1_position = jntstate.position[0]
    sensor_msg.servo2_position = jntstate.position[1]
    sensor_msg.servo1_current = jntstate.effort[0]
    sensor_msg.servo2_current = jntstate.effort[1]
    pub_sensor.publish(sensor_msg)
    rawsensors_buffer.append(msg)
    sensors_buffer.append(sensor_msg)
    
    #state_cmd = StateCommand()
    #state_cmd.header = msg.header
    #state_cmd.joint_angle = sensor_msg.joint_angle
    #state_cmd.servo1_position = sensor_msg.servo1_position
    #state_cmd.servo2_position = sensor_msg.servo2_position
    #cmd = cmd_buffer[-1]
    #state_cmd.u1 = cmd.u1
    #state_cmd.u2 = cmd.u2
    #state_cmd.u3 = cmd.u3
    #pub_statecmd.publish(state_cmd)
    if len(sensors_buffer) > 500:
        del sensors_buffer[0]
        del rawsensors_buffer[0]


def sub_rawcmd_cb(msg):
    cmd = Command()
    cmd.u1 = model.servo1_usec2rad(msg.u1)
    cmd.u2 = model.servo2_usec2rad(msg.u2)
    cmd.u3 = msg.u3/255
    cmd_buffer.append(cmd)
    rawcmd_buffer.append(msg)
    if len(cmd_buffer) > 100:
        del cmd_buffer[0]
        del rawcmd_buffer[0]


if __name__ == '__main__':
    sensors_buffer = []
    rawsensors_buffer = []
    #rawcmd_buffer = []
    #cmd_buffer = []
    #cmd0 = Command()
    #cmd0.u1 = 0
    #cmd0.u2 = 0
    #cmd0.u3 = 0
    #cmd_buffer.append(cmd0)
    model = MaccepavdModel()
    rospy.init_node('call_sensors_server')
    print('node init')
    sub_rawssr = message_filters.Subscriber('/sensors_raw', SensorsRawAdc)
    sub_js = message_filters.Subscriber('/dynamixel_workbench/joint_states', JointState)
    ts = message_filters.ApproximateTimeSynchronizer([sub_rawssr, sub_js], 10, 0.1)
    ts.registerCallback(sub_sensors_cb)
    print('message filter init')
    #sub_rawcmd = rospy.Subscriber('command_raw', CommandRaw, sub_rawcmd_cb)
    #pub_statecmd = rospy.Publisher('state_command', StateCommand, queue_size = 10)
    pub_sensor = rospy.Publisher('sensors', Sensors, queue_size = 10)
    #service = rospy.Service('call_sensors', CallSensors, call_sensor_cb)
    #service_raw = rospy.Service('call_rawsensors', CallRawSensors, call_rawsensor_cb)
    rospy.spin()
