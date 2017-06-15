#! /usr/bin/env python

import rospy
import actionlib
from maccepavd.msg import ExecuteTrajectoryFeedbackAction, ExecuteTrajectoryFeedbackGoal, \
    ExecuteTrajectoryFeedbackFeedback, ExecuteTrajectoryFeedbackResult, CommandRaw

class ExeTjFeedbackAS(object):
    _feedback = ExecuteTrajectoryFeedbackFeedback()
    _result = ExecuteTrajectoryFeedbackResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                ExecuteTrajectoryFeedbackAction,self.exetj_cb,False)
        self._as.start()

    def exetj_cb(self,goal):
        # todo: to complete
        nominal_commands = goal.nominal_commands
        control_rate = rospy.Rate(goal.control_frequence)
        n_steps = goal.length
        k = 0
        while (k < n_steps):
            u = [nominal_commands[k].u1;nominal_commands[k].u2;nominal_commands[k].u3]
            cmd_to_send =


if __name__ == '__main__':
    rospy.init_node('exetj_feedback_action_server')
    server = ExeTjFeedbackAS(rospy.get_name())
    rospy.spin()
