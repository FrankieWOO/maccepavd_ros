#! /usr/bin/env python

import sys
import time
import math
from numpy import *
import pyrex_gui     as gui
from maccepavd.msg import CommandRaw


def direct_motor_control():
	"""Control motors directly with sliders."""
	slb = gui.createSliderBox(3,'Direct Motor Control')
    u_llim = [-1.0472, 0, 0]
    u_ulim = [1.0472, 2.094, 1]
    for i in range(0,3):
		slb.setupSlider(i,u_llim[i],u_ulim[i],'u%d'%i,0.01)
	slb.setValues([0,0,0]) # set sliders to zeros

	while slb.checkExit() == 0:
		u = slb.getValues()
		y = robot.run_step(u)
		qdot = (y[0]-q)/0.02; q = y[0]; qddot = y[1]; m[0]=y[2]; m[1]=y[3];
		q_scope    .add([q])
		qdot_scope .add([qdot])
		qddot_scope.add([qddot])
		m0_scope   .add([u[0],m[0]])
		m1_scope   .add([u[1],m[1]])


if __name__ == "__main__":
    rospy.init_node('motor_control')
    pub_rawcmd = rospy.Publisher('command_raw', CommandRaw, queue_size=2)
    robot_model = MaccepavdModel()
