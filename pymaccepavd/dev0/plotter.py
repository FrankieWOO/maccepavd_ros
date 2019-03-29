#! /usr/bin/env python

import sys
import rospy
import numpy as np
from maccepavd_model import MaccepavdModel
from mccpvdgui import Ui_mccpvdgui, ControlWindow
from maccepavd.msg import StateCommand
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore


def sub_cb(msg):
    pass

if __name__ == '__main__':
    model = MaccepavdModel()
    rospy.init_node('plotter')
    
    sub = rospy.Subscriber('state_command', StateCommand, sub_cb)
    rospy.spin()
