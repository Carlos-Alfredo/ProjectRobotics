import sim
import time
import math
import numpy as np
from Robot_FW import Robot_FW
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5);
print(clientID)
if clientID!=0:
	print('Conexão com problema')	
else:
	Robot = Robot_FW(clientID,'rollingJoint')
	#Robot.arm_position([0,0,0,0])
	rC,p=sim.simxGetJointPosition(clientID,Robot.joint2,sim.simx_opmode_streaming)
	time.sleep(2)
	rC,q_1=sim.simxGetJointPosition(clientID,Robot.joint2,sim.simx_opmode_buffer)
	Robot.arm_position([0,-90,0,-90])