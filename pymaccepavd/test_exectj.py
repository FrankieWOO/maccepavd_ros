#! /usr/bin/env python

import glob
import os, sys
import scipy.io, numpy, rospy, actionlib
import pandas as pd
import matplotlib.pyplot as plt
import subprocess
from maccepavd.msg import ExecuteTrajectoryForwardAction, ExecuteTrajectoryForwardGoal, ExecuteTrajectoryForwardResult, Command
#from maccepavd_model import MaccepavdModel
from record_ros.srv import String_cmd
from datetime import datetime


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


def exectj_forward(filename):
    filename = 'tjlib/'+filename
    data = scipy.io.loadmat(filename)
    if data.keys()[0] == 'result':
        res = data['result']
        val = res[0,0]
        u = val['u']
        Nu = u.shape[1]

    # create the Goal
    goal = ExecuteTrajectoryForwardGoal()
    goal.control_frequency = 50
    goal.commands = []
    for i in range(Nu):
        cmd = Command()
        unow = u[0:3,i]
        cmd.u1 = unow[0]
        cmd.u2 = unow[1]
        cmd.u3 = unow[2]
        goal.commands.append(cmd)
    client_forward.send_goal(goal)
    client_forward.wait_for_result()
    # todo: exit program if action completed according to result
    print ('action completed')


def exectj_plot(filename):
    filename = 'tjlib/'+filename
    data = scipy.io.loadmat(filename)
    if data.keys()[0] == 'result':
        res = data['result']
        val = res[0,0]
        u = val['u']
        x = val['x']
        Nu = u.shape[1]

    # create the Goal
    goal = ExecuteTrajectoryForwardGoal()
    goal.control_frequency = 50
    goal.commands = []
    for i in range(Nu):
        cmd = Command()
        unow = u[0:3,i]
        cmd.u1 = unow[0]
        cmd.u2 = unow[1]
        cmd.u3 = unow[2]
        goal.commands.append(cmd)
    recorder('record')
    # todo: continue after the rosbag node starts recording
    rospy.sleep(0.5)
    client_forward.send_goal(goal)
    client_forward.wait_for_result()
    rospy.sleep(0.1)
    try:
        recorder('stop')
    except Exception:
        pass
    print ('action completed')
    # todo: continue program after the rosbag node stop recording to avoid missing the bagfile
    rospy.sleep(0.5)
    file_path = os.path.join('/home/fan/rosbag_record/','*.bag')
    files = glob.glob(file_path)
    thefile = max(files, key = os.path.getctime)
    timenow = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    csvfilename = '/home/fan/rosbag_record/recordtj_' + timenow + '.csv'
    tmpcsvssr = '/home/fan/rosbag_record/tmp_sensors.csv'
    tmpcsvrcmd = '/home/fan/rosbag_record/tmp_rawcmd.csv'

    subprocess.call('rostopic echo -p -b {} /sensors > {}'.format(thefile,tmpcsvssr), shell = True)
    subprocess.call('rostopic echo -p -b {} /command_raw > {}'.format(thefile,tmpcsvrcmd), shell = True)
    subprocess.call('rostopic echo -p -b {} /state_command > {}'.format(thefile,csvfilename), shell = True)
    #bag = rosbag.Bag(thefile)
    #for topic, msg, t in bag.read_messages(topics=[''])
    datassr = pd.read_csv(tmpcsvssr)
    timestamps = datassr.iloc[:,2]
    xm = datassr.iloc[:,4]
    xservo1 = datassr.iloc[:,5]
    timestamps = (timestamps - timestamps[0])*(10**-9)
    datassr.iloc[:,2] = timestamps
    datassr.to_csv(tmpcsvssr, index=False)
    tsim = numpy.asarray(range(Nu+1))*0.02
    plot_tj(u, x, tsim, xm, timestamps, xservo1)
    #dataread = pd.read_csv(csvfilename)
    #timestamps = dataread.iloc[:,2]
    #timestamps = (timestamps - timestamps[0])*(10**-9)
    #dataread.iloc[:,2] = timestamps
    #dataread.to_csv(csvfilename, index=False)

def plot_tj(u, xsim, tsim, xm, tm, xservo1):
    fig, axs = plt.subplots(3,1, sharex = True)
    axs[0].plot(tsim,xsim[0,:],tm,xm, tm, xservo1)
    axs[1].plot(tsim[0:-1], u[0,:], tsim[0:-1], u[1,:])
    axs[2].plot(tsim[0:-1],u[2,:])
    plt.show()
    #legend()

if __name__ == '__main__':
    rospy.init_node('test_exectj')
    # create the action client, the name must match the name used when creating the action server
    client_forward = actionlib.SimpleActionClient('exectj_forward', ExecuteTrajectoryForwardAction)
    # wait for action server to be ready
    client_forward.wait_for_server()
    #model = MaccepavdModel()
    recorder = rospy.ServiceProxy('/record/cmd', String_cmd)

    if len(sys.argv) != 1:
        if sys.argv[1] == 'exec_plot':
            filename = sys.argv[2]
            exectj_plot(filename)
        elif sys.argv[1] == 'exec':
            exectj_forward(filename)
    else:
        #print('no trajectory file given')
        test_exectjforward()
