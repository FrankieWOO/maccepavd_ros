#! /usr/bin/env python

import rospy, actionlib
from maccepavd.msg import ExecuteTrajectoryForwardAction, ExecuteTrajectoryForwardGoal,\
    ExecuteTrajectoryForwardFeedback, ExecuteTrajectoryForwardResult, CommandRaw, Sensors
from maccepavd.srv import CallSensors, CallSensorsRequest, CallSensorsResponse
from maccepavd_model import MaccepavdModel




class ExecTjForwardAS(object):
    _feedback = ExecuteTrajectoryForwardFeedback()
    _result = ExecuteTrajectoryForwardResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                ExecuteTrajectoryForwardAction, self.exectj_cb, False)
        self._as.start()
        self._model = MaccepavdModel()


    def exectj_cb(self,goal):
        global record_flag, record_buffer
        commands = goal.commands
        control_rate = rospy.Rate(goal.control_frequency)
        n_steps = len(commands)
        rawcmd = []
        for i in range(len(commands)):
            rawcmd.append(self._model.cmd2raw(commands[i]))

        arecord = []
        record_flag = True
        for i in range(len(commands)):
            #feedback = ExecuteTrajectoryForwardFeedback()
            pub_rawcmd.publish(rawcmd[i])
            self._feedback = ExecuteTrajectoryForwardFeedback()
            #
            #resp = request_state(1)
            #self._feedback.sensors = resp.sensors
            #arecord.append(self._feedback.sensors)
            #
            #self._feedback.sensors = record_buffer[-1]
            self._as.publish_feedback(self._feedback)
            control_rate.sleep()
        #
        #resp = request_state(1)
        #arecord.append(resp.sensors)
        record_flag = False
        arecord = record_buffer
        record_buffer = []
        aresult = ExecuteTrajectoryForwardResult()
        aresult.record = arecord
        aresult.complete = True
        self._as.set_succeeded(result=aresult)


def sub_sensors_cb(msg):
    global record_flag, record_buffer
    if record_flag:
        record_buffer.append(msg)


if __name__ == '__main__':
    rospy.init_node('exectj_forward_action_server')
    server = ExecTjForwardAS('exectj_forward')
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)
    request_state = rospy.ServiceProxy('call_sensors', CallSensors)
    record_flag = False
    record_buffer = []
    sub_ssr = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    rospy.spin()
