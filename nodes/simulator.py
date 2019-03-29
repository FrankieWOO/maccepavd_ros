#! /usr/bin/env python
import rospy
from maccepavd.msg import SensorsRawAdc, CommandRaw


def cmdcb(cmdmsg):
    print('command received:{},{},{},{}'.format(cmdmsg.u1,cmdmsg.u2,cmdmsg.D1,cmdmsg.D2))


if __name__ == '__main__':
    rospy.init_node('simMaccepa')
    rospy.Subscriber('command_raw', CommandRaw, cmdcb)
    pub = rospy.Publisher('sensors_raw', SensorsRawAdc, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ssrmsg = SensorsRawAdc()
        ssrmsg.joint_sensor = 1500
        ssrmsg.servo1_sensor = 400
        ssrmsg.servo2_sensor = 400
        ssrmsg.rege_current = 0.0
        ssrmsg.servo_current = 0.0
        pub.publish(ssrmsg)
        rate.sleep()
