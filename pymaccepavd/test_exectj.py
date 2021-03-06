#! /usr/bin/env python

import glob
import numpy as np
import os, sys
import scipy.io, numpy, rospy, actionlib
import pandas as pd
import matplotlib.pyplot as plt
import subprocess
from maccepavd.msg import ExecuteTrajectoryForwardAction, ExecuteTrajectoryForwardGoal, \
ExecuteTrajectoryForwardResult, Command, Sensors
#from maccepavd_model import MaccepavdModel
#from record_ros.srv import String_cmd
from maccepavd.srv import StartRecord, StopRecord
from datetime import datetime


def sub_sensors_cb(msg):
    global record
    if record:
        record_buffer.append(msg)


def convert_msglist_to_dict(msgs):
    fieldnames = msgs[0].__slots__
    data = {x: [] for x in fieldnames} # init a dict of empty lists
    for k in range(len(msgs)):
        for f in fieldnames:
            data[f].append(getattr(msgs[k], f))

    if 'header' in fieldnames:
        for k in range(len(data['header'])):
            data['header'][k] = data['header'][k].stamp.to_sec()

    return data


def test_exectjforward():
    global record, record_buffer
    # test the action server
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
    #start_record_proxy(1)

    record = True
    client_forward.send_goal(goal)
    client_forward.wait_for_result()
    #record_response = stop_record_proxy(1)
    #record_res = record_response.sensors
    # todo: exit program if action completed according to result
    action_result = client_forward.get_result()
    if action_result.complete:
        print ('action completed')
    else:
        print('action not completed')
    action_record = action_result.record
    t0 = rospy.get_time()
    record = False
    #y = convert_msglist_to_dict(action_record) # retrieve the list of msg and convert to dict
    y = convert_msglist_to_dict(record_buffer)
    record_buffer = []
    y['header'] = np.asarray(y['header']) - t0

    #rege_current = []
    #servo_current = []
    #time_stamp = []
    #for ssrmsg in record_res:
    #    q.append(ssrmsg.joint_position)
        #rege_current.append(ssrmsg.rege_current)
        #servo_current.append(ssrmsg.servo_current)
    #    time_stamp.append(ssrmsg.header.stamp.toSec())
    print(len(y['header']))
    #print(action_record)
    print(len(action_record))


def exectj_forward(filename):
    #execute mat file
    filename = 'tjlib/'+filename
    data = scipy.io.loadmat(filename)
    datakeys = data.keys()
    for key in datakeys:
        if 'result' in key:
            res = data[key]
            val = res[0,0]
            u = val['u']
            Nu = u.shape[1]
            break

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
    start_record_proxy(1)
    client_forward.send_goal(goal)
    client_forward.wait_for_result()
    # todo: exit program if action completed according to result
    record_response = stop_record_proxy(1)
    record_res = record_response.sensors
    q = []
    rege_current = []
    servo_current = []
    time_stamp = []
    for ssrmsg in record_res:
        q.append(ssrmsg.joint_position)
        rege_current.append(ssrmsg.rege_current)
        servo_current.append(ssrmsg.servo_current)
        time_stamp.append(ssrmsg.header.stamp.toSec())

    #time_stamp = (time_stamp - time_stamp[0])*(10**-9)
    #fig, axs = plt.subplots(3,1, sharex = True)
    #axs[0].plot(time_stamp, q )
    #axs[1].plot(time_stamp, rege_current)
    #axs[2].plot(time_stamp, servo_current)
    #plt.show()
    print(q)
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
    #rospy.sleep(0.5)
    client_forward.send_goal(goal)
    tstart = rospy.get_time()
    client_forward.wait_for_result()
    #rospy.sleep(0.1)
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

    subprocess.call('rostopic echo -p -b {} /sensors_raw > {}'.format(thefile,tmpcsvssr), shell = True)
    subprocess.call('rostopic echo -p -b {} /command_raw > {}'.format(thefile,tmpcsvrcmd), shell = True)
    subprocess.call('rostopic echo -p -b {} /state_command > {}'.format(thefile,csvfilename), shell = True)
    #bag = rosbag.Bag(thefile)
    #for topic, msg, t in bag.read_messages(topics=[''])
    datassr = pd.read_csv(tmpcsvssr)
    index_start = 0
    timestamps = datassr.iloc[index_start:,2]
    xm = datassr.iloc[index_start:,4]
    xservo1 = datassr.iloc[index_start:,5]
    timestamps = (timestamps - timestamps[0])*(10**-9)
    datassr.iloc[index_start:,2] = timestamps
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
    axs[0].plot(tsim,xsim[0,:],tm,xm )
    axs[1].plot(tsim[0:-1], u[0,:], tsim[0:-1], u[1,:],tm, xservo1)
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
    #recorder = rospy.ServiceProxy('/record/cmd', String_cmd)
    #start_record_proxy = rospy.ServiceProxy('/maccepavd/start_record', StartRecord)
    #stop_record_proxy = rospy.ServiceProxy('/maccepavd/stop_record', StopRecord)
    sub_rawssr = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    record = False
    record_buffer = []
    if len(sys.argv) != 1:
        if sys.argv[1] == 'exec_plot':
            filename = sys.argv[2]
            exectj_plot(filename)
        elif sys.argv[1] == 'exec':
            filename = sys.argv[2]
            exectj_forward(filename)
    else:
        #print('no trajectory file given')
        test_exectjforward()
