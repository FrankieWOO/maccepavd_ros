#! /usr/bin/env python

import rospy, message_filters
from maccepavd_model import MaccepavdModel
from maccepavd.msg import SensorsRaw, Sensors, CommandRaw, Command
from maccepavd.srv import CallSensors, CallSensorsRequest, CallSensorsResponse, CallRawSensors, CallRawSensorsRequest, CallRawSensorsResponse


def call_sensor_cb(request):
    # todo: return n sensors msg
    # n = request.n
    return CallSensorsResponse(sensors_buffer[-1])


def call_rawsensor_cb(request):
    return CallRawSensorsResponse(rawsensors_buffer[-1])


def sub_sensors_cb(msg):
    # convert and store the sensor reading into buffer
    sensor_msg = model.raw2sensors(msg)
    rawsensors_buffer.append(msg)
    sensors_buffer.append(sensor_msg)
    if len(sensors_buffer) > 5000:
        del sensors_buffer[0]


if __name__ == '__main__':
    sensors_buffer = []
    rawsensors_buffer = []
    model = MaccepavdModel()
    rospy.init_node('call_sensors_server')
    sub_rawssr = rospy.Subscriber('sensors_raw', SensorsRaw, sub_sensors_cb)
    #sub_rawcmd = message_filters.Subscriber('command_raw', CommandRaw, )
    service = rospy.Service('call_sensors', CallSensors, call_sensor_cb)
    service_raw = rospy.Service('call_rawsensors', CallRawSensors, call_rawsensor_cb)
    rospy.spin()
