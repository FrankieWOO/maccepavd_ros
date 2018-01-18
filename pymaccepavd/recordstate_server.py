#! /usr/bin/env python

import rospy
from maccepavd.msg import Sensors, StartRecord, StartRecordRequest, StartRecordResponse, StopRecord, StopRecordRequest, StopRecordResponse


def sub_sensors_cb(sensors_msg):
    if to_record == True:
        state_buffer.append(sensors_msg)

def start_record_cb(request):
    to_record = True

def stop_record_cb(request):
    to_record = False
    return StopRecordResponse(state_buffer)


if __name__ == '__main__':
    state_buffer = []
    to_record = False
    rospy.init_node('record_state_server')
    sub_sensors = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    service_start_record = rospy.Service('/maccepavd/start_record', StartRecord, start_record_cb)
    service_stop_record = rospy.Service('/maccepavd/stop_record', StopRecord, stop_record_cb)
    rospy.spin()
