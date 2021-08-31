import sim
import time
import math
import numpy as np
from Robot_FW import Robot_FW
from inverse_kin import OptimizationFunction

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5);
if clientID!=0:
	print('Conexão com problema')	
else:

	Robot = Robot_FW(clientID,'rollingJoint')


	res_1, cuboid0Handle=sim.simxGetObjectHandle(clientID,"Cuboid",sim.simx_opmode_blocking)
	res_2, sensorHandle = sim.simxGetObjectHandle(clientID,"sensor_1",sim.simx_opmode_blocking)
	res_3, dummyHandle1=sim.simxGetObjectHandle(clientID,"Dummy1",sim.simx_opmode_blocking)
	res_4, dummyHandle2=sim.simxGetObjectHandle(clientID,"Dummy2",sim.simx_opmode_blocking)
	res_5, dummyHandle3=sim.simxGetObjectHandle(clientID,"Dummy3",sim.simx_opmode_blocking)

	r=sim.simxReadProximitySensor(clientID,sensorHandle,sim.simx_opmode_streaming)
	r=sim.simxReadProximitySensor(clientID,sensorHandle,sim.simx_opmode_buffer)
	Robot.open_grip(True)
	
	

	while True:
		Robot.pick_up(cuboid0Handle,sensorHandle,1)
		
		#Placing
		#Robot.put_down(dummyHandle1,1)
		#Robot.put_down(dummyHandle2,2)
		break
