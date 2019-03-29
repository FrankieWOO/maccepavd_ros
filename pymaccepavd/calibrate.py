#! /usr/bin/env python

import rospy, rosbag, sys, os, subprocess, math
import pandas as pd
import numpy as np
from maccepavd.msg import CommandRaw, SensorsRawAdc, Command, StateCommand
from maccepavd.srv import CallSensors, CallRawSensors
from record_ros.srv import String_cmd
from maccepavd_model import MaccepavdModel
from datetime import datetime


def calibrate_joint():
    global record, record_buffer
    record = True
    rospy.sleep(0.5)
    record = False
    y = record_buffer
    record_buffer = []
    p0 = []
    for r in y:
        p0.append(r.joint_sensor)
    p0 = np.mean(np.asarray(p0))
    pub_rawcmd.publish(1800, 1500, 255 ,0)
    rospy.sleep(2.)
    record = True
    rospy.sleep(0.5)
    record = False
    y = record_buffer
    record_buffer = []
    p1 = []
    for r in y:
        p1.append(r.joint_sensor)
    p1 = np.mean(np.asarray(p1))
    w = math.pi/6/(p1 - p0)
    off = -w*p0;
    print("{}, {}".format(w, off))


def calibrate_servo1(deg):
    pub_rawcmd.publish(1500, 900, 0, 0)
    global record, record_buffer
    rospy.sleep(2.0)
    record = True
    rospy.sleep(0.5)
    record = False
    y = record_buffer
    record_buffer = []
    s1 = []
    for r in y:
        s1.append(r.servo1_sensor)
    s1 = np.mean(np.asarray(s1))

    pub_rawcmd.publish(1800, 900, 0, 0)
    rospy.sleep(2.0)
    record = True
    rospy.sleep(0.5)
    record = False
    y = record_buffer
    record_buffer = []
    s2 = []
    for r in y:
        s2.append(r.servo1_sensor)
    s2 = np.mean(np.asarray(s2))
    """
    0 = w*s1 + off
    pi/6 = w*s2 + off
    """
    w = math.pi/6/(s2 - s1)
    off = -w*s1
    print("{}, {}".format(w, off))



def calibrate_servo2(deg):
    u_deg = [deg, 0, deg, 0, deg, 0, deg, 0]
    Nu = len(u_deg)
    recorder('record')
    rospy.sleep(0.01)
    for u in u_deg:
        u1 = model.servo1_deg2usec(0)
        u2 = model.servo2_deg2usec(u)
        u3 = model.dc2adc(0)
        pub_rawcmd.publish(u1, u2, u3)
        rospy.sleep(0.5)
    recorder('stop')


def step_response_servo1_old(deg):
    # run 1 command and collect sensor data
    bagfile = os.path.dirname(os.path.abspath(__file__)) +'/log/test_servo1.bag'
    timenow = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    csvfilename = os.path.dirname(os.path.abspath(__file__))+'/log/servo1_step{}_'.format(deg) + timenow + '.csv'
    bag = rosbag.Bag(bagfile, 'w')
    u1 = model.servo1_deg2usec(deg)
    u2 = model.servo2_deg2usec(0)
    u3 = model.dc2adc(0)
    start_time = rospy.get_time()
    zero_cmd = Command()
    zero_cmd.u1 = 0
    zero_cmd.u2 = 0
    zero_cmd.u3 = 0
    cmd = Command()
    cmd.u1 = math.radians(deg)
    cmd.u2 = 0
    cmd.u3 = 0
    while rospy.get_time() - start_time <= 0.2:
        rawsensors_response = rawsensors_caller(1)
        rawsensors_msg = rawsensors_response.sensors_raw
        sensors_response = sensors_caller(1)
        sensors_msg = sensors_response.sensors
        #print(rawsensors_msg.servo1_sensor)
        #print(sensors_msg.joint_angle)
        statecmd = StateCommand()
        statecmd.header = sensors_msg.header
        statecmd.joint_angle = sensors_msg.joint_angle
        statecmd.servo1_position = sensors_msg.servo1_position
        statecmd.servo2_position = sensors_msg.servo2_position
        statecmd.u1 = zero_cmd.u1
        statecmd.u2 = zero_cmd.u2
        statecmd.u3 = zero_cmd.u3
        t1 = rospy.get_rostime()
        bag.write('state_command', statecmd, t1)
        bag.write('sensors_raw', sensors_msg, t1)
        r.sleep()

    pub_rawcmd.publish(u1, u2, u3)
    #rospy.sleep(0.001)
    while rospy.get_time() - start_time <= 2:
        rawsensors_response = rawsensors_caller(1)
        rawsensors_msg = rawsensors_response.sensors_raw
        sensors_response = sensors_caller(1)
        sensors_msg = sensors_response.sensors
        statecmd = StateCommand()
        statecmd.header = sensors_msg.header
        statecmd.joint_angle = sensors_msg.joint_angle
        statecmd.servo1_position = sensors_msg.servo1_position
        statecmd.servo2_position = sensors_msg.servo2_position
        statecmd.u1 = cmd.u1
        statecmd.u2 = cmd.u2
        statecmd.u3 = cmd.u3
        t1 = rospy.get_rostime()
        bag.write('state_command', statecmd, t1)
        bag.write('sensors_raw', sensors_msg, t1)
        r.sleep()
    bag.close()
    subprocess.call('rostopic echo -p -b {} /state_command > {}'.format(bagfile,csvfilename), shell = True)
    dataread = pd.read_csv(csvfilename)
    timestamps = dataread.iloc[:,2]
    timestamps = (timestamps - timestamps[0])*(10**-9)
    dataread.iloc[:,2] = timestamps
    dataread.to_csv(csvfilename,index=False)


def step_response_servo2_old(deg):
    # run 1 command and collect sensor data
    timenow = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    csvfilename = os.path.dirname(os.path.abspath(__file__))+'/log/servo2_step{}_'.format(deg) + timenow + '.csv'
    bagfile = os.path.dirname(os.path.abspath(__file__))+'/log/test_servo2.bag'
    bag = rosbag.Bag(bagfile, 'w')
    u1 = model.servo1_deg2usec(0)
    u2 = model.servo2_deg2usec(deg)
    u3 = model.dc2adc(0)
    start_time = rospy.get_time()
    zero_cmd = Command()
    zero_cmd.u1 = 0
    zero_cmd.u2 = 0
    zero_cmd.u3 = 0
    cmd = Command()
    cmd.u1 = 0
    cmd.u2 = math.radians(deg)
    cmd.u3 = 0
    while rospy.get_time() - start_time <= 0.2:
        rawsensors_response = rawsensors_caller(1)
        rawsensors_msg = rawsensors_response.sensors_raw
        sensors_response = sensors_caller(1)
        sensors_msg = sensors_response.sensors
        #print(rawsensors_msg.servo1_sensor)
        #print(sensors_msg.joint_angle)
        statecmd = StateCommand()
        statecmd.header = sensors_msg.header
        statecmd.joint_angle = sensors_msg.joint_angle
        statecmd.servo1_position = sensors_msg.servo1_position
        statecmd.servo2_position = sensors_msg.servo2_position
        statecmd.u1 = zero_cmd.u1
        statecmd.u2 = zero_cmd.u2
        statecmd.u3 = zero_cmd.u3
        t1 = rospy.get_rostime()
        bag.write('state_command', statecmd, t1)
        bag.write('sensors_raw', sensors_msg, t1)
        r.sleep()

    pub_rawcmd.publish(u1, u2, u3)
    #rospy.sleep(0.001)
    while rospy.get_time() - start_time <= 2:
        rawsensors_response = rawsensors_caller(1)
        rawsensors_msg = rawsensors_response.sensors_raw
        sensors_response = sensors_caller(1)
        sensors_msg = sensors_response.sensors
        statecmd = StateCommand()
        statecmd.header = sensors_msg.header
        statecmd.joint_angle = sensors_msg.joint_angle
        statecmd.servo1_position = sensors_msg.servo1_position
        statecmd.servo2_position = sensors_msg.servo2_position
        statecmd.u1 = cmd.u1
        statecmd.u2 = cmd.u2
        statecmd.u3 = cmd.u3
        t1 = rospy.get_rostime()
        bag.write('state_command', statecmd, t1)
        bag.write('sensors_raw', sensors_msg, t1)
        r.sleep()
    bag.close()
    subprocess.call('rostopic echo -p -b {} /state_command > {}'.format(bagfile,csvfilename), shell = True)
    dataread = pd.read_csv(csvfilename)
    timestamps = dataread.iloc[:, 2]
    timestamps = (timestamps - timestamps[0]) * (10 ** -9)
    dataread.iloc[:, 2] = timestamps
    dataread.to_csv(csvfilename, index=False)


def go_to_zeros():
    cmd = CommandRaw()
    u1 = 1500
    u2 = 650
    u3 = 0
    pub_rawcmd.publish(u1,u2,u3)
    rospy.sleep(0.1)


#def shutdown_hook():
#    print("Shutdown node")

def mean_readings():
    jnt = []
    servo1 = []
    servo2 = []
    mc = []
    s1c = []
    s2c = []
    for i in range(0,50):
        rawsensors_response = rawsensors_caller(1)
        rawsensors_msg = rawsensors_response.sensors_raw
        jnt.append(rawsensors_msg.joint_sensor)
        servo1.append(rawsensors_msg.servo1_sensor)
        servo2.append(rawsensors_msg.servo2_sensor)
        mc.append(rawsensors_msg.motor_current)
        s1c.append(rawsensors_msg.servo1_current)
        s2c.append(rawsensors_msg.servo2_current)
        r.sleep()
    jnt = numpy.mean(jnt)
    servo1 = numpy.mean(servo1)
    servo2 = numpy.mean(servo2)
    mc = numpy.mean(mc)
    s1c = numpy.mean(s1c)
    s2c = numpy.mean(s2c)
    print("{}, {}, {}, {}, {}, {}".format(jnt,servo1,servo2,mc,s1c,s2c))


def sub_sensors_cb(msg):
    if record:
        record_buffer.append(msg)


if __name__ == '__main__':
    model = MaccepavdModel()
    rospy.init_node('calibrate')
    #rospy.wait_for_service('call_sensors')
    #rawsensors_caller = rospy.ServiceProxy('call_rawsensors', CallRawSensors)
    #sensors_caller = rospy.ServiceProxy('call_sensors', CallSensors)
    #recorder = rospy.ServiceProxy('/record/cmd', String_cmd)
    record = False
    record_buffer = []
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=2)
    sub_rawssr = rospy.Subscriber('sensors_raw', SensorsRawAdc, sub_sensors_cb)

    r = rospy.Rate(100)
    if sys.argv[1] == '0':
        go_to_zeros()
    elif sys.argv[1] == 'servo1':
        if len(sys.argv) == 2:
            calibrate_servo1(30)
        else: calibrate_servo1(float(sys.argv[2]))
    elif sys.argv[1] == 'servo2':
        if len(sys.argv) == 2:
            calibrate_servo2(45)
        else: calibrate_servo2(float(sys.argv[2]))
    elif sys.argv[1] == 'readings':
        mean_readings()
    elif sys.argv[1] == 'joint':
        calibrate_joint()
    else: print('wrong input arguments')
