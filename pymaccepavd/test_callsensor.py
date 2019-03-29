#! /usr/bin/env python

import rospy
from maccepavd.msg import Sensors
from maccepavd.srv import CallSensors, CallSensorsRequest, CallSensorsResponse


if __name__ == "__main__":
    request_state = rospy.ServiceProxy('call_sensors', CallSensors)
    resp = request_state(1)
    print(resp.sensors)
    print(resp)
