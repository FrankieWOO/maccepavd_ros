#! /usr/bin/env python

import rospy, actionlib
from maccepavd.msg import ExecuteTrajectoryForwardAction, ExecuteTrajectoryForwardGoal,\
    ExecuteTrajectoryForwardFeedback, ExecuteTrajectoryForwardResult, CommandRaw
from maccepavd_model import MaccepavdModel


class ExecTjForwardAS(object):
    _feedback = ExecuteTrajectoryForwardFeedback()
    _result = ExecuteTrajectoryForwardResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                ExecuteTrajectoryForwardAction,self.exectj_cb,False)
        self._as.start()
        self._model = MaccepavdModel()

    def exectj_cb(self,goal):
        commands = goal.commands
        control_rate = rospy.Rate(goal.control_frequency)
        n_steps = len(commands)
        rawcmd = []
        for i in range(len(commands)):
            rawcmd.append(self._model.cmd2raw(commands[i]))

        for i in range(len(commands)):
            #feedback = ExecuteTrajectoryForwardFeedback()
            pub_rawcmd.publish(rawcmd[i])
            control_rate.sleep()

        result = ExecuteTrajectoryForwardResult()
        result.complete = True
        self._as.set_succeeded(result=result)

if __name__ == '__main__':
    rospy.init_node('exectj_forward_action_server')
    server = ExecTjForwardAS('exectj_forward')
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)
    rospy.spin()
