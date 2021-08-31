import sim
import numpy as np
import time

sim.simxFinish(-1) 
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) 
print(clientID)

if clientID!=-1:
    RC_dict = {}
    returnCode1,joint1 = sim.simxGetObjectHandle(clientID,'YBAJ0',sim.simx_opmode_blocking)
    
    returnCode2,joint2 = sim.simxGetObjectHandle(clientID,'YBAJ1',sim.simx_opmode_blocking)
    returnCode3,joint3 = sim.simxGetObjectHandle(clientID,'YBAJ2',sim.simx_opmode_blocking)
    
    returnCode4,joint4 = sim.simxGetObjectHandle(clientID,'YBAJ3',sim.simx_opmode_blocking)
    returnCode5,joint5 = sim.simxGetObjectHandle(clientID,'YBAJ4',sim.simx_opmode_blocking)

    returnCode6,eixo_rl = sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking)
    returnCode7,eixo_rr = sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking)
    returnCode8,eixo_fl = sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking)
    returnCode9,eixo_fr = sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking)

    rC_vector = [returnCode1,returnCode2,returnCode3,
                 returnCode4,returnCode5,returnCode6,returnCode7,returnCode8,returnCode9]
    
    jointAngles1 = np.arange(0,140,20)
    jointAngles2 = np.arange(0,140,20)

    time.sleep(5)

    

    
    
    #print([RC1,RC2,RC3,RC4])
    for index in range(len(jointAngles1)):
        sim.simxPauseCommunication(clientID,True)
        
        """

        sim.simxSetJointTargetPosition(clientID, joint1, 
                                            jointAngles1[index]*np.pi/180, 
                                            sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID, joint5, 
                                            jointAngles1[index]*np.pi/180, 
                                            sim.simx_opmode_oneshot)

        """

        RC1=sim.simxSetJointTargetVelocity(clientID, eixo_rl,10*np.pi/180, 
                                            sim.simx_opmode_oneshot)
        RC2=sim.simxSetJointTargetVelocity(clientID, eixo_rr, 
                                            10*np.pi/180, 
                                            sim.simx_opmode_oneshot)
        RC3=sim.simxSetJointTargetVelocity(clientID, eixo_fl, 
                                            10*np.pi/180, 
                                            sim.simx_opmode_oneshot)
        RC4=sim.simxSetJointTargetVelocity(clientID, eixo_fr, 
                                            10*np.pi/180, 
                                            sim.simx_opmode_oneshot)
        #returnCode3 = sim.simxPauseCommunication(clientID,False)        
        #print(f'FIRST JOINT: {returncode1}')
        returnCode3 = sim.simxPauseCommunication(clientID,False)

    time.sleep(15)    
    sim.simxPauseCommunication(clientID,True)
    
    returnCode3 = sim.simxPauseCommunication(clientID,False)
else:
    print("Deu ruim a conexão");


