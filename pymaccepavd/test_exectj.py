#! /usr/bin/env python

import scipy.io, sys, numpy, rospy, actionlib
from maccepavd.msg import ExecuteTrajectoryForwardAction, ExecuteTrajectoryForwardGoal, ExecuteTrajectoryForwardResult, Command


def test_exectjforward():
    u1 = numpy.ones(100)*0.7
    u2 = numpy.zeros_like(u1)
    u3 = numpy.zeros_like(u1)
    # create the Goal
    goal = ExecuteTrajectoryForwardGoal()
    goal.control_frequency = 50
    goal.commands = []
    for i in range(100):
        cmd = Command()
        cmd.u1 = u1[i]
        cmd.u2 = u2[i]
        cmd.u3 = u3[i]
        goal.commands.append(cmd)
    client_forward.send_goal(goal)
    client_forward.wait_for_result()
    # todo: exit program if action completed according to result
    print ('action completed')


if __name__ == '__main__':
    rospy.init_node('test_exectj')
    # create the action client, the name must match the name used when creating the action server
    client_forward = actionlib.SimpleActionClient('exectj_forward', ExecuteTrajectoryForwardAction)
    # wait for action server to be ready
    client_forward.wait_for_server()

    if len(sys.argv) != 1:
        filename = sys.argv[1]
        test_exectjforward()
    else:
        #print('no trajectory file given')
        test_exectjforward()