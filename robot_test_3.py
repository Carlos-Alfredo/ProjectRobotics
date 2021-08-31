import sim
import time
import math
import numpy as np
from Robot_FW import Robot_FW
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5);
if clientID!=0:
	print('Conexão com problema')	
else:
	Robot = Robot_FW(clientID,'rollingJoint')
	