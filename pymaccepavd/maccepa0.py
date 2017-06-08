## \file test_maccepa.py
#  \author Matthew Howard (MH), matthew.howard@kcl.ac.uk
#  \ingroup MACCEPA
#  \brief Script for testing control of the MACCEPA.

import sys
import time
import math
from numpy import *
import pyrex_gui     as gui
import pyrex_maccepa as maccepa
#import pyrex_audio   as audio
import matplotlib.pyplot as plt


def go_zeros():
	"""Move motors to zero position."""
	u=[]
	for i in range(0,robot.dimU): u.append(0)
	robot.write(u)


def step_response():
	u= zeros([3,1])
	u[0]=1
	u[1]=0
	u[2]=0
	N = 100
	states = zeros([N,3])
	for n in range(0,N):
		y0=robot.write(u)
		y = robot.read_adc()
		print("{},{},{}".format(y[0],y[1],y[2]) )
		states[n,0] = y[0]
		states[n,1] = y[1]
		states[n,2] = y[2]
		time.sleep(0.02)
	t = range(0,N)
	plt.plot(t,states[:,1])
	plt.show()

def execute_test_trajectory():
	""" Execute some sinusoidal trajectory open-loop on the robot to test that the interface is working properly.
	"""
	N = 500
	u = zeros([robot.dimU,N])
	for n in range(0,N):
		u[0,n]=.1*math.pi* math.sin(1e-2*n);
		u[1,n]=.3*math.pi*(math.sin(5e-2*n-math.pi/2)+1);
	robot.write(u[:,1])
	time.sleep(3)

	for n in range(0,N):
		y=robot.run_step(u[:,n])
		time.sleep(0.02)
	go_zeros()

def direct_motor_control():
	"""Control motors directly with sliders."""
	slb = gui.createSliderBox(robot.dimU,'Direct Motor Control')
	for i in range(0,robot.dimU):
		slb.setupSlider(i,robot.u_llim[i],robot.u_ulim[i],'u%d'%i,0.01)
	y=robot.read(); qdot = 0; q = y[0]; qddot = y[1] # read arm state
	slb.setValues([0,0,0]) # set sliders to zeros

	# scope for plotting out sensor signals
	q_scope     = gui.Scope(1,  0,250,600,200,'Position')
	qdot_scope  = gui.Scope(1,  0,475,600,200,'Velocity')
	qddot_scope = gui.Scope(1,  0,700,600,200,'Acceleration')
	m0_scope    = gui.Scope(2,680,250,600,200,'Motor 0 Commanded/Actual Positions')
	m1_scope    = gui.Scope(2,680,475,600,200,'Motor 1 Commanded/Actual Positions')

	m = [0,0]
	while slb.checkExit() == 0:
		u = slb.getValues()
		y = robot.run_step(u)
		qdot = (y[0]-q)/0.02; q = y[0]; qddot = y[1]; m[0]=y[2]; m[1]=y[3];
		q_scope    .add([q])
		qdot_scope .add([qdot])
		qddot_scope.add([qddot])
		m0_scope   .add([u[0],m[0]])
		m1_scope   .add([u[1],m[1]])

	# return to zero position when done
	go_zeros()

def online_plot():
	# scope for plotting out sensor signals
	q_scope     = gui.Scope(1,  0,250,600,200,'Position')
	qdot_scope  = gui.Scope(1,  0,475,600,200,'Velocity')
	qddot_scope = gui.Scope(1,  0,700,600,200,'Acceleration')
	m1_scope    = gui.Scope(2,680,250,600,200,'Motor 1 Commanded/Actual Positions')
	m2_scope    = gui.Scope(2,680,475,600,200,'Motor 2 Commanded/Actual Positions')
	m = [0,0]
	while q_scope.checkExit() == 0:
	#u = slb.getValues()
		y = robot.read()
		qdot = (y[0]-q)/0.02; q = y[0]; qddot = y[1]; m[0]=y[2]; m[1]=y[3];
		q_scope    .add([q])
		qdot_scope .add([qdot])
		qddot_scope.add([qddot])
		m0_scope   .add([u[0],m[0]])
		m1_scope   .add([u[1],m[1]])


def test_servo1():
	useq = [0.5, 1, 1.5, 1, 0.5 ,0, -0.5 , -1, -1.5 , -0.5,0 ,0.5, 1, 1.5, 1, 0.5 ,0, -0.5 , -1, -1.5 , -0.5,0]
	"""Control motors directly with sliders."""
	slb = gui.createSliderBox(robot.dimU,'Direct Motor Control')
	for i in range(0,robot.dimU):
		slb.setupSlider(i,robot.u_llim[i],robot.u_ulim[i],'u%d'%i,0.01)
	y=robot.read(); qdot = 0; q = y[0]; qddot = y[1] # read arm state
	slb.setValues([0,0,0]) # set sliders to zeros

	# scope for plotting out sensor signals
	q_scope     = gui.Scope(1,  0,250,600,200,'Position')
	qdot_scope  = gui.Scope(1,  0,475,600,200,'Velocity')
	qddot_scope = gui.Scope(1,  0,700,600,200,'Acceleration')
	m0_scope    = gui.Scope(2,680,250,600,200,'Motor 0 Commanded/Actual Positions')
	m1_scope    = gui.Scope(2,680,475,600,200,'Motor 1 Commanded/Actual Positions')

	m = [0,0]

	for n in range(0,22):
		u = slb.getValues()
		u[0] = useq[n]
		y = robot.run_step(u)
		qdot = (y[0]-q)/0.02; q = y[0]; qddot = y[1]; m[0]=y[2]; m[1]=y[3];
		q_scope    .add([q])
		qdot_scope .add([qdot])
		qddot_scope.add([qddot])
		m0_scope   .add([u[0],m[0]])
		m1_scope   .add([u[1],m[1]])
		time.sleep(1)

	# return to zero position when done
	go_zeros()

def emg_control():
	"""Control motors directly with EMG."""
	slb = gui.createSliderBox(10,'EMG Eq. Pos./Stiffness Control')
	slb.setupSlider(0, 0,2000,'EMG High Pass'     ,0.1)
	slb.setupSlider(1, 0,2000,'EMG Low Pass'      ,0.1)
	slb.setupSlider(2, 0,10  ,'EMG Smoothing'     ,0.01)
	slb.setupSlider(3, 1,10  ,'EMG0 gain   (red)' ,0.1)
	slb.setupSlider(4,-1,10  ,'EMG0 offset (red)' ,0.0001)
	slb.setupSlider(5, 1,10  ,'EMG1 gain   (blue)',0.1)
	slb.setupSlider(6,-1,10  ,'EMG1 offset (blue)',0.0001)
	slb.setupSlider(7, 0,50  ,'u0 gain'           ,0.1)
	slb.setupSlider(8, 0,50  ,'u1 gain'           ,0.1)
	slb.setupSlider(9,-1,1   ,'Normal / Reverse'  ,1)
	slb.setValues([60, 600, 2.24, 2.1, 0, 2.9, 0, 0, 0, 1])

        emg = audio.AudioInterface()

	emg_scope = gui.Scope(2,620,0,600,200,'EMG Signals')
	u0_scope  = gui.Scope(2,620,250,600,200,'Desired/actual equilibrium position')
	u1_scope  = gui.Scope(2,620,475,600,200,'Desired/actual stiffness')

	u1min = .01

	a  = zeros([2,1])
	u  = zeros([robot.dimU,1])
	ud = zeros([robot.dimU,1])
	while slb.checkExit()==0:
		v = slb.getValues()
		emg.setHP(v[0])
		emg.setLP(v[1])
		emg.setSmooth(v[2])

		a = list(emg.getData())
		a[0] = v[3]*a[0] + v[4];
		a[1] = v[5]*a[1] + v[6];

		if len(a)>0:
			emg_scope.add(a)

			# estimate desired eq. pos and stiffness
			ud[0] = v[9]*v[7]*( a[0] - a[1]) # u0 (= q0) -> scaled difference between signals
			ud[1] = v[8]*((a[0] + a[1] - math.fabs(a[0] - a[1]))/2)+u1min # u1 (= k) -> min of the two signals

			u = robot.clip_commands(ud)

			y = robot.run_step(u)
			u0_scope.add([ud[0],u[0]])
			u1_scope.add([ud[1],u[1]])


if __name__ == "__main__":
   if len(sys.argv) != 2:
       print "Please specify the USB port to which control board is attached (e.g., /dev/ttyACM0)."
       exit()
   else:
       robot = maccepa.HardwareInterface(sys.argv[1])
       model = maccepa.ModelInterface()
