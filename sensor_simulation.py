
import sim
import numpy as np
import time
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) 

if clientID!=-1:
    returnCode1,joint1 = sim.simxGetObjectHandle(clientID,'',sim.simx_opmode_blocking)