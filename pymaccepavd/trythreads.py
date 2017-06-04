#! /usr/bin/env python

import threading, rospy, time
from maccepavd.msg import CommandRaw, SensorsRaw, Command, StateCommand


class ros_node(threading.Thread):
    def __init__(self, n):
        super(ros_node, self).__init__()
        #rospy.init_node('mynode{}'.format(n))
        print('mynode{}'.format(n))
        #rospy.spin()
        self.total = 0
        self.stop = False

    def run(self):
        while not self.stop:
            self.total+= 1

def new_node(n):
    rospy.init_node('mymynode{}'.format(n))
    print('node{}'.format(n))
    rospy.spin()


if __name__ == "__main__":
    stcmd = StateCommand()
    stcmd.joint_angle = 1
    print(stcmd.joint_angle)
    #node2.start()
    #rospy.spin()
