#! /usr/bin/env python

import numpy as np
import math
import pandas as pd
import os, sys, glob
python3 = True if sys.hexversion > 0x03000000 else False
import rospy
import matplotlib.pyplot as plt
from maccepavd.msg import Command, CommandRaw, Sensors
from maccepavd_model import MaccepavdModel


def sub_sensors_cb(msg):
    if record:
        record_buffer.append(msg)

def exec_single_traj(traj):
    """
    input: traj - numpy ndarray, or dict
    output: y - dict
    """
    global record, record_buffer
    cmds = []
    if 'numpy' in str(type(traj)):
        n = traj.shape[1]
        for i in range(n):
            cmd = Command()
            cmd.u1 = traj[0,i]
            cmd.u2 = traj[1,i]
            cmd.u3 = traj[2,i]
            cmds.append(model.cmd2raw(cmd))
    elif 'dict' in str(type(traj)):
        for i in range(len(traj['u1'])):
            cmd = Command()
            cmd.u1 = traj['u1'][i]
            cmd.u2 = traj['u2'][i]
            cmd.u3 = traj['u3'][i]
            cmds.append(model.cmd2raw(cmd))

    loop_rate = rospy.Rate(50)
    #t0 = rospy.get_rostime().to_sec()
    t0 = rospy.get_time()
    record = True # turn on data record
    for j in range(len(cmds)):
        pub_rawcmd.publish(cmds[j])
        loop_rate.sleep()
    record = False # turn off data record

    y = convert_msglist_to_dict(record_buffer) # retrieve the list of msg and convert to dict
    record_buffer = []
    y['header'] = np.asarray(y['header']) - t0
    return y


def exec_traj_csv(filename):
    """
    read traj from csv and execute it
    return y as dict
    """
    df = pd.read_csv(filename)
    traj = df.to_dict()
    y = exec_single_traj(traj)
    return y


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


def save_dict_csv(data, filename):
    df = pd.DataFrame.from_dict(data)
    df.to_csv(filename)


def test():
    #folderpath = '/home/fan/workspace/EOC/matlab/trajs/test/'
    #trajfile = 'test_reach.csv'
    #savefile = 'test_record.csv'

    folderpath = 'trajs/test/'
    trajfile = 'test_slow.csv'
    y = exec_traj_csv(folderpath+trajfile)
    plt.plot(y['header'],y['servo1_position'])
    plt.plot(y['header'],y['joint_position'])
    #plt.plot(y['header'],y['rege_current'])

    plt.show()
    #save_dict_csv(y, 'record/vardamp_test/'+savefile)


def vardamp(iter):
    """
    execute trajs in vardamp folder
    """
    #folderpath = '/home/fan/workspace/EOC/matlab/trajs/vardamp/command/'
    #filenames = glob.glob(folderpath+'*.csv')
    #savefolder = '/home/fan/workspace/EOC/matlab/trajs/vardamp/record/'
    folderpath = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs'+ str(iter)+'/vardamp/command/'
    filenames = glob.glob(folderpath+'*.csv')
    savefolder = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs/vardamp/record/'
    for k in range(len(filenames)):
        y = exec_traj_csv(folderpath+'traj'+str(k+1)+'.csv')
        save_dict_csv(y, savefolder+'record_traj'+str(k+1)+'_'+str(iter)+'.csv')
        rospy.sleep(0.1)


def fixdamp(iter):
    """
    execute trajs in vardamp folder
    """
    #folderpath = '/home/fan/workspace/EOC/matlab/trajs/fixdamp/command/'
    #filenames = glob.glob(folderpath+'*.csv')
    #savefolder = '/home/fan/workspace/EOC/matlab/trajs/fixdamp/record/'
    folderpath = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs'+ str(iter)+'/fixdamp/command/'
    filenames = glob.glob(folderpath+'*.csv')
    savefolder = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs/fixdamp/record/'

    for k in range(len(filenames)):
        y = exec_traj_csv(folderpath+'traj'+str(k+1)+'.csv')
        save_dict_csv(y, savefolder+'record_traj'+str(k+1)+'_'+str(iter)+'.csv')
        rospy.sleep(0.1)


def multitrajs_optimal(iter):
    """
    execute trajs in optimal folder
    """
    folderpath = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs'+ str(iter)+'/optimal/command/'
    filenames = glob.glob(folderpath+'*.csv')
    savefolder = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs/optimal/record/'
    for k in range(len(filenames)):
        y = exec_traj_csv(folderpath+'traj'+str(k+1)+'.csv')
        save_dict_csv(y, savefolder+'record_traj'+str(k+1)+'_'+str(iter)+'.csv')
        rospy.sleep(0.1)


def constant(iter):
    """
    execute trajs in constant folder
    """
    folderpath = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs'+str(iter)+'/constant/command/'
    filenames = glob.glob(folderpath+'*.csv')

    #savefolder = '/home/fan/workspace/EOC/matlab/trajs/constant/record/'
    savefolder = '/home/fan/catkin_ws/src/maccepavd/pymaccepavd/trajs/trajs/constant/record/'
    for k in range(len(filenames)):
        y = exec_traj_csv(folderpath+'traj'+str(k+1)+'.csv')
        save_dict_csv(y, savefolder+'record_traj'+str(k+1)+'_'+str(iter)+'.csv')
        rospy.sleep(0.1)



if __name__ == '__main__':
    rospy.init_node('exectraj')
    record = False
    record_buffer = []
    model = MaccepavdModel()
    cmd0 = Command()
    cmd0.u1 = 0
    #cmd0.u2 = math.pi/6
    cmd0.u2 = 0.1
    cmd0.u3 = 0
    rawcmd0 = model.cmd2raw(cmd0)
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)
    sub_rawssr = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    pub_rawcmd.publish(rawcmd0)
    rospy.sleep(1.)
    #test()
    #vardamp(2)
    #constant(2)
    #multitrajs_optimal(2)
    #fixdamp(2)
    if len(sys.argv) != 1:
        if sys.argv[1] == 'exec_plot':
            filename = sys.argv[2]
            exectj_plot(filename)
        elif sys.argv[1] == 'exec':
            filename = sys.argv[2]
            y = exec_traj_csv('/home/fan/Dropbox/workspace/ws_matlab/redundancy_control/research/PTOCA/data/ilqrseq/command/Df.csv')
            save_dict_csv(y, '/home/fan/Dropbox/workspace/ws_matlab/redundancy_control/research/PTOCA/data/ilqrseq/command/Df_record.csv')
        elif sys.argv[1] == 'optimal':
            multitrajs_optimal(1)
    else:
        #print('no trajectory file given')
        test_exectjforward()

    rospy.sleep(0.1)
    pub_rawcmd.publish(rawcmd0)
    rospy.sleep(2.0)
