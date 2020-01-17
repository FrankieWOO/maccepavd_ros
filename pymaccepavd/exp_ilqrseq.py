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
from dynamixel_workbench_msgs.srv import DynamixelCommandTwo, DynamixelCommandTwoRequest, DynamixelCommandTwoResponse
from dynamixel_workbench_msgs.msg import Dxlcmd2


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


def setup_dxl():
	# Control table address
    ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_GOAL_POSITION      = 116
    ADDR_PRESENT_POSITION   = 132

    # Data Byte Length
    LEN_GOAL_POSITION       = 4
    LEN_PRESENT_POSITION    = 4

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL1_ID                     = 0                 # Dynamixel#1 ID : 1
    DXL2_ID                     = 1                 # Dynamixel#1 ID : 2
    BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    #DXL_MINIMUM_POSITION_VALUE  = 100           # Dynamixel will rotate between this value
    #DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupBulkWrite instance
    groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

    # Initialize GroupBulkRead instace for Present Position
    groupBulkRead = GroupBulkRead(portHandler, packetHandler)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


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
        dxlcmd = Dxlcmd2()
        dxlcmd.id1 = 0
        dxlcmd.id2 = 1
	dxlcmd.value1 = cmds[j].u1
	dxlcmd.value2 = cmds[j].u2
	dxlcmd.addr_name = 'Goal_Position'
        pub_dxlcmd2.publish(dxlcmd)
	#srv_cmddxl('',0,1,'Goal_Position',cmds[j].u1,cmds[j].u2)
	#srv_cmddxl('',1,'Goal_Position',cmds[j].u2)
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


def d0_minEin():
    #folderpath = '/home/fan/workspace/EOC/matlab/trajs/test/'
    #trajfile = 'test_reach.csv'
    #savefile = 'test_record.csv'
    #curpath = os.path.realpath(__file__)
    #print(curpath)
    folderpath = '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/'
    trajfile = 'D0_minEin.csv'
    y = exec_traj_csv(folderpath+trajfile)
    print( sum( [abs(y1) for y1 in y['servo1_current'] ] + [abs(y2) for y2 in y['servo2_current'] ] ) ) 
    save_dict_csv(y, '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/D0_minEin_record.csv')
    df = pd.read_csv(folderpath+trajfile)
    plt.figure(0)
    plt.plot(df["t"],df["u1"])
    plt.plot(y['header'], y['joint_position'])    
    plt.plot(y['header'], y['servo1_position'])
    plt.plot(y['header'], y['servo2_position'])
        
    plt.figure(1)
    plt.plot(y['header'], y['servo1_current'])
    plt.plot(y['header'], y['servo2_current'])
    plt.show()
    #plt.plot(y['header'],y['rege_current'])


def df_minEin():
    folderpath = '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/'
    trajfile = 'Df_minEin.csv'
    y = exec_traj_csv(folderpath+trajfile)
    print( sum( [abs(y1) for y1 in y['servo1_current'] ] + [abs(y2) for y2 in y['servo2_current'] ] ) ) 
    save_dict_csv(y, '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/Df_minEin_record.csv')
    df = pd.read_csv(folderpath+trajfile)
    plt.figure(0)
    plt.plot(df["t"],df["u1"])
    plt.plot(y['header'], y['joint_position'])    
    plt.plot(y['header'], y['servo1_position'])
    plt.plot(y['header'], y['servo2_position'])
        
    plt.figure(1)
    plt.plot(y['header'], y['servo1_current'])
    plt.plot(y['header'], y['servo2_current'])
    plt.show()


def d0_minEin_origin():
    #folderpath = '/home/fan/workspace/EOC/matlab/trajs/test/'
    #trajfile = 'test_reach.csv'
    #savefile = 'test_record.csv'
    #curpath = os.path.realpath(__file__)
    #print(curpath)
    folderpath = '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/'
    trajfile = 'D0_minEin_origin.csv'
    y = exec_traj_csv(folderpath+trajfile)
    print( sum( [abs(y1) for y1 in y['servo1_current'] ] + [abs(y2) for y2 in y['servo2_current'] ] ) ) 
    save_dict_csv(y, '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/D0_minEin_origin_record.csv')
    df = pd.read_csv(folderpath+trajfile)
    plt.figure(0)
    plt.plot(df["t"],df["u1"])
    plt.plot(y['header'], y['joint_position'])    
    plt.plot(y['header'], y['servo1_position'])
    plt.plot(y['header'], y['servo2_position'])
        
    plt.figure(1)
    plt.plot(y['header'], y['servo1_current'])
    plt.plot(y['header'], y['servo2_current'])
    plt.show()
    #plt.plot(y['header'],y['rege_current'])


def df_minEin_origin():
    folderpath = '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/'
    trajfile = 'Df_minEin_origin.csv'
    y = exec_traj_csv(folderpath+trajfile)
    print( sum( [abs(y1) for y1 in y['servo1_current'] ] + [abs(y2) for y2 in y['servo2_current'] ] ) ) 
    save_dict_csv(y, '/home/fan/Dropbox/workspace/ws_matlab/compliant_arm_control/research/PTOCA/data/ilqrseq/command/Df_minEin_origin_record.csv')
    df = pd.read_csv(folderpath+trajfile)
    plt.figure(0)
    plt.plot(df["t"],df["u1"])
    plt.plot(y['header'], y['joint_position'])    
    plt.plot(y['header'], y['servo1_position'])
    plt.plot(y['header'], y['servo2_position'])
        
    plt.figure(1)
    plt.plot(y['header'], y['servo1_current'])
    plt.plot(y['header'], y['servo2_current'])
    plt.show()


if __name__ == '__main__':
    rospy.init_node('exectraj')
    record = False
    record_buffer = []
    model = MaccepavdModel()
    cmd0 = Command()
    cmd0.u1 = 0
    #cmd0.u2 = math.pi/6
    cmd0.u2 = 0.0
    cmd0.u3 = 0
    rawcmd0 = model.cmd2raw(cmd0)
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=10)	
    pub_dxlcmd2 = rospy.Publisher('/dynamixel_workbench/dxlcmd2', Dxlcmd2, queue_size =10)
    sub_rawssr = rospy.Subscriber('sensors', Sensors, sub_sensors_cb)
    srv_cmddxl = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command2',DynamixelCommandTwo)
    # reset to zero position
    pub_rawcmd.publish(rawcmd0)
    srv_cmddxl('',0,1,'Goal_Position',2048,597)
    #srv_cmddxl('',1,'Goal_Position',512)
    rospy.sleep(2.0)
    if len(sys.argv) != 1:
    	if sys.argv[1] == 'd0_minEin':
            d0_minEin()
	    # reset to zero position
            pub_rawcmd.publish(rawcmd0)
            srv_cmddxl('',0,1,'Goal_Position',2048,597)
        elif sys.argv[1] == 'df_minEin':
	    df_minEin()
            # reset to zero position
            pub_rawcmd.publish(rawcmd0)
            srv_cmddxl('',0,1,'Goal_Position',2048,597)
	elif sys.argv[1] == 'd0_minEin_origin':
            d0_minEin_origin()
            # reset to zero position
            pub_rawcmd.publish(rawcmd0)
            srv_cmddxl('',0,1,'Goal_Position',2048,597)
	elif sys.argv[1] == 'df_minEin_origin':
            df_minEin_origin()
            # reset to zero position
            pub_rawcmd.publish(rawcmd0)
            srv_cmddxl('',0,1,'Goal_Position',2048,597)
                    
    else:
        print('no trajectory file given')
        #test()

    #rospy.sleep(0.1)
    #pub_rawcmd.publish(rawcmd0)
    #rospy.sleep(1.0)
