# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
    import math
    import numpy as np
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim




if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)

    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(5)
    #jointArm1 = "redundantRob_joint1"
    #jointArm2 = "redundantRob_joint2"
    #jointArm3 = "redundantRob_joint3"
    #jointArm4 = "redundantRob_joint4"
    
    returnCode1,joint1 = sim.simxGetObjectHandle(clientID,'MTB_axis1',sim.simx_opmode_blocking)
    returnCode2,joint2 = sim.simxGetObjectHandle(clientID,'MTB_axis2',sim.simx_opmode_blocking)
    print(joint1)

    jointAngles1 = np.arange(0,40,10)
    jointAngles2 = np.arange(0,40,10)
   
    for index in range(len(jointAngles1)):
        returnCode4= sim.simxPauseCommunication(clientID,True)
        returncode1 = sim.simxSetJointPosition(clientID, joint1, 
                                               math.radians(jointAngles1[index]), 
                                               sim.simx_opmode_oneshot)
        print(returncode1)
        for i in range(len(jointAngles2)):
            returncode2 = sim.simxSetJointPosition(clientID, joint2, 
                                                   math.radians(jointAngles2[index]), 
                                                   sim.simx_opmode_oneshot)
            print(f'SECOND JOINT: {returncode2}')
        returnCode3 = sim.simxPauseCommunication(clientID,False)
        
        time.sleep(1)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

