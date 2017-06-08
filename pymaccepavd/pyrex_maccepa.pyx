## \file maccepa_pyrex_interface.pyx
#  \author Matthew Howard (MH), matthew.howard@ed.ac.uk
#  \ingroup MACCEPA
#  \brief Pyrex interface for controlling/reading sensor values from the MACCEPA.

DEF DIMQ = 1 # TODO: how to automatically extract these values from sketchbook/maccepa/defines.h ?
#DEF DIMU = 3 #
DEF DIMU = 3 #
DEF DIMY = 5 #
DEF DIMX = 2*DIMQ

cdef extern from "../sketchbook/maccepa/defines.h":
	double U_ULIM_RAD_SERVO0
	double U_LLIM_RAD_SERVO0
	double U_ULIM_RAD_SERVO1
	double U_LLIM_RAD_SERVO1
	double U_ULIM_DAMPER0
	double U_LLIM_DAMPER0

cdef extern from "vsa_arduino_interface.h":
	ctypedef struct ArduinoInterface:
		pass
	int  vsa_arduino_interface_init       ( ArduinoInterface *AI, char *device )
	void vsa_arduino_interface_close      ( ArduinoInterface *AI )
	void vsa_arduino_interface_write      ( ArduinoInterface *AI, double *u )
	void vsa_arduino_interface_write_usec ( ArduinoInterface *AI, int *u )
	void vsa_arduino_interface_read       ( ArduinoInterface *AI, double *y )
	void vsa_arduino_interface_read_adc   ( ArduinoInterface *AI, int *y )
	void vsa_arduino_interface_run_step   ( ArduinoInterface *AI, double *u, double *y )

cdef extern from "math.h":
	double M_PI
	double fabs(double)

cdef extern from "libmaccepa.h":
	ctypedef struct maccepa_model:
		double inertia

	void maccepa_model_init                              ( maccepa_model * model )
	void maccepa_model_get_torque                        ( double * tau, double * x, double * u, maccepa_model * model )
	void maccepa_model_get_equilibrium_position          ( double *  q0, double * x, double * u, maccepa_model * model )
	void maccepa_model_get_equilibrium_position_jacobian ( double *   J, double * x, double * u, maccepa_model * model )
	void maccepa_model_get_stiffness                     ( double *   k, double * x, double * u, maccepa_model * model )
	void maccepa_model_get_stiffness_jacobian            ( double *   J, double * x, double * u, maccepa_model * model )

cdef class ModelInterface:
	"""A class for making dynamics calculations based on a model of the 1-DoF MACCEPA."""

	cdef maccepa_model Model

	def __init__(self):
		maccepa_model_init(&self.Model)

	def getTorque(self,x,u):
		"""
		tau = getTorque(x,u)

		Calculate torque due to command u in state x.
		"""
		cdef double   cx[DIMX]
		cdef double   cu[DIMU]
		cdef double ctau[DIMQ]
		for i in range(0,DIMX): cx[i] = x[i] # copy to c array
		for i in range(0,DIMU): cu[i] = u[i] # 
		maccepa_model_get_torque(ctau,cx,cu,&self.Model)
		return ctau[0]

	def getStiffness(self,x,u):
		"""
		k = getStiffness(x,u)

		Calculate joint stiffness due to command u in state x.
		"""
		cdef double cx[DIMX]
		cdef double cu[DIMU]
		cdef double ck[DIMQ]
		for i in range(0,DIMX): cx[i] = x[i] # copy to c array
		for i in range(0,DIMU): cu[i] = u[i] # 
		maccepa_model_get_stiffness(ck,cx,cu,&self.Model)
		return ck[0]

	def getStiffnessJacobian(self,x,u):
		"""J = getStiffnessJacobian(x,u)

		   Calculate Jacobian of joint stiffness with respect to motor commands
		   for a given command u and state x.

		"""
		cdef double cx[DIMX]
		cdef double cu[DIMU]
		cdef double cJ[DIMU]
		J = []
		for i in range(0,DIMX): cx[i] = x[i] # copy to c array
		for i in range(0,DIMU): cu[i] = u[i] # 
		maccepa_model_get_stiffness_jacobian(cJ,cx,cu,&self.Model)
		for i in range(0,DIMU): J.append(cJ[i]) # copy to python array
		return J

	def getEquilibriumPosition(self,x,u):
		"""q0 = getEquilibriumPosition(x,u)

		   Calculate joint equilibrium position due to command u in state x.

		"""
		cdef double cx[DIMX]
		cdef double cu[DIMU]
		cdef double cq0[DIMQ]
		for i in range(0,DIMX): cx[i] = x[i] # copy to c array
		for i in range(0,DIMU): cu[i] = u[i] # 
		maccepa_model_get_equilibrium_position(cq0,cx,cu,&self.Model)
		return cq0[0]

	def getEquilibriumPositionJacobian(self,x,u):
		"""J = getEquilibriumPositionJacobian(x,u)

		   Calculate Jacobian of joint equilibrium position with respect to motor
		   commands for a given command u and state x.

		"""
		cdef double cx[DIMX]
		cdef double cu[DIMU]
		cdef double cJ[DIMU]
		J = []
		for i in range(0,DIMX): cx[i] = x[i] # copy to c array
		for i in range(0,DIMU): cu[i] = u[i] # 
		maccepa_model_get_equilibrium_position_jacobian(cJ,cx,cu,&self.Model)
		for i in range(0,DIMU): J.append(cJ[i]) # copy to python array
		return J


cdef class HardwareInterface:
	"""Hardware interface to the 1-DoF MACCEPA (through the Arduino Duemilanove 328)."""
	cdef ArduinoInterface AI
	cdef int isOk
	dimU = DIMU
	u_ulim=[U_ULIM_RAD_SERVO0,U_ULIM_RAD_SERVO1,U_ULIM_DAMPER0]
	u_llim=[U_LLIM_RAD_SERVO0,U_LLIM_RAD_SERVO1,U_LLIM_DAMPER0]
	#u_ulim=[U_ULIM_RAD_SERVO0,U_ULIM_RAD_SERVO1]
	#u_llim=[U_LLIM_RAD_SERVO0,U_LLIM_RAD_SERVO1]

	def __init__(self, port):
		self.isOk = vsa_arduino_interface_init(&self.AI, port)

	def __dealloc__(self): 
		vsa_arduino_interface_close(&self.AI)

	def run_step(self,u):
		"""pos,acc,m1,m2,power,timestamp = run_step(u0, u1)

		   Read position, acceleration, motor position sensors and timestamp
		   from the robot, and command new positions u0, u1.

		   This function blocks so that the sensor readings and commands are synchronised.
		"""
		cdef double cu[DIMU]  
		cdef double cy[DIMY+1]
		y = []
		if len(u)<DIMU: 
			print "Command must be a list of double values of length %d." % DIMU
			return 
		for i in range(0,DIMU): cu[i] = u[i] # copy command to c array
		vsa_arduino_interface_run_step(&self.AI, cu, cy)
		for i in range(0,DIMY+1): y.append(cy[i]) # copy to python array
		return y # return sensor readings

	def read(self):
		"""
		   pos,acc,m1,m2,power,timestamp = read()

		   Read position, acceleration, and motor positions from the robot.

		   Calling this function will block until the next frame of data has arrived from the Arduino.
		"""
		cdef double cy[DIMY+1]
		y=[]; 
		vsa_arduino_interface_read(&self.AI, cy)
		for i in range(0,DIMY+1): y.append(cy[i]) # copy to python array
		return y

	def read_adc(self):
		""" 
		   pos,acc,m1,m2,power = read()

		   Read sensor signals pos,acc,m0,m1 in adc steps (i.e., raw voltages). 
		"""
		cdef int cy[DIMY+1]
		y=[];
		vsa_arduino_interface_read_adc(&self.AI, cy)
		for i in range(0,DIMY+1): y.append(cy[i]) # copy to python array
		return y

	def write(self, u): 
		"""write(u)

		   Command new motor positions u0, u1.

		   This function will NOT block, and there is no guarantee
		   about when the new positions will be sent to the servos.
		"""
		if len(u)<DIMU: 
			print "Command must be a list of double values of length %d." % DIMU
			return 
		cdef double cu[DIMU]
		for i in range(0,DIMU): cu[i] = u[i] # 
		vsa_arduino_interface_write(&self.AI, cu)

	def write_usec(self, u): 
		"""write_usec(u0, u1)

		   Command new motor positions u0, u1 in units of 0.5 microseconds.

		   Note: 

		 	1. This function will NOT block, and there is no guarantee
			   about when the new positions will be sent to the servos. 

			2. This function will not limit commands to a fixed range!

		   This function is only for configuring the servos, and not intended for normal control.
		"""
		if len(u)<DIMU: 
			print "Command must be a list of integer values of length %d." % DIMU
			return 
		cdef int cu[DIMU]
		for i in range(0,DIMU): cu[i] = u[i] # 
		vsa_arduino_interface_write_usec(&self.AI,cu)

	def clip_commands(self,u):
		if u[0]>U_ULIM_RAD_SERVO0:
			u[0]=U_ULIM_RAD_SERVO0
		elif u[0]<U_LLIM_RAD_SERVO0:
			u[0]=U_LLIM_RAD_SERVO0
		if u[1]>U_ULIM_RAD_SERVO1:
			u[1]=U_ULIM_RAD_SERVO1
		elif u[1]<U_LLIM_RAD_SERVO1:
			u[1]=U_LLIM_RAD_SERVO1
		if u[2]>U_ULIM_DAMPER0:
			u[2]=U_ULIM_DAMPER0
		elif u[2]<U_LLIM_DAMPER0:
			u[2]=U_LLIM_DAMPER0
		return u

