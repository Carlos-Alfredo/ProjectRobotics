import sim
import time
import math

#CONSTANTS
jointArm1 = "redundantRob_joint1"
jointArm2 = "redundantRob_joint2"
jointArm3 = "redundantRob_joint3"
jointArm4 = "redundantRob_joint4"
jointArm5 = "redundantRob_joint5"
jointArm6 = "redundantRob_joint6"

link5Arm = "redundantRob_link5"

eofArm = "Rectangle4"

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
print(clientID)

if clientID!=-1:
    print ('Connected to remote API server')

    # Start the simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait)

    returnCode,joint1 = sim.simxGetObjectHandle(clientID,jointArm1,sim.simx_opmode_blocking)
    returnCode,joint2 = sim.simxGetObjectHandle(clientID,jointArm2,sim.simx_opmode_blocking)
    returnCode,joint3 = sim.simxGetObjectHandle(clientID,jointArm3,sim.simx_opmode_blocking)
    returnCode,joint4 = sim.simxGetObjectHandle(clientID,jointArm4,sim.simx_opmode_blocking)
    returnCode,joint5 = sim.simxGetObjectHandle(clientID,jointArm5,sim.simx_opmode_blocking)
    returnCode,joint6 = sim.simxGetObjectHandle(clientID,jointArm6,sim.simx_opmode_blocking)

    
    jointAngles1 = [0,0,0,30,30,30,0,30,30,-10,-10]
    jointAngles2 = [0,60,30,30,60,30,30,40,30,30,40]
    jointAngles4 = [0,40,30,30,40,30,30,40,30,30,40]
    jointAngles6 = [0,70,30,30,70,30,30,70,30,30,70]

    for index in range(11):
        print('oi')
        returnCode = sim.simxPauseCommunication(clientID,True)
        returncode = sim.simxSetJointTargetPosition(clientID, joint1, math.radians(jointAngles1[index]), sim.simx_opmode_oneshot)
        returncode = sim.simxSetJointTargetPosition(clientID, joint2, math.radians(jointAngles2[index]), sim.simx_opmode_oneshot)
        returncode = sim.simxSetJointTargetPosition(clientID, joint4, math.radians(jointAngles4[index]), sim.simx_opmode_oneshot)
        returncode = sim.simxSetJointTargetPosition(clientID, joint6, math.radians(jointAngles6[index]), sim.simx_opmode_oneshot)
        returnCode = sim.simxPauseCommunication(clientID,False)
        time.sleep(1)

    sim.simxFinish(clientID)