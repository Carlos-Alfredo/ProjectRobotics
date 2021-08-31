import sim
import time
import math
import numpy as np
from inverse_kin import OptimizationFunction

class Robot_FW():
	def __init__(self,clientID,RobotName):
		self.clientID = clientID
		returnCode1,self.joint1 = sim.simxGetObjectHandle(clientID,'YBAJ0',sim.simx_opmode_blocking)
		returnCode2,self.joint2 = sim.simxGetObjectHandle(clientID,'YBAJ1',sim.simx_opmode_blocking)
		returnCode3,self.joint3 = sim.simxGetObjectHandle(clientID,'YBAJ2',sim.simx_opmode_blocking)
		returnCode4,self.joint4 = sim.simxGetObjectHandle(clientID,'YBAJ3',sim.simx_opmode_blocking)
		returnCode5,self.joint5 = sim.simxGetObjectHandle(clientID,'YBAJ4',sim.simx_opmode_blocking)
		returnCode5,self.gjoint1 = sim.simxGetObjectHandle(clientID,'youBotGripperJoint1',sim.simx_opmode_blocking)	
		returnCode5,self.gjoint2 = sim.simxGetObjectHandle(clientID,'youBotGripperJoint2',sim.simx_opmode_blocking)	
		
		returnCode6,self.eixo_rl = sim.simxGetObjectHandle(clientID,f'{RobotName}_rl',sim.simx_opmode_blocking)
		returnCode7,self.eixo_rr = sim.simxGetObjectHandle(clientID,f'{RobotName}_rr',sim.simx_opmode_blocking)
		returnCode8,self.eixo_fl = sim.simxGetObjectHandle(clientID,f'{RobotName}_fl',sim.simx_opmode_blocking)
		returnCode9,self.eixo_fr = sim.simxGetObjectHandle(clientID,f'{RobotName}_fr',sim.simx_opmode_blocking)
		

		print([returnCode1,returnCode2,returnCode3,returnCode4,
			  returnCode6,returnCode7,returnCode8,returnCode9])
	
	def velocity(self,v):
		[rl,rr,fl,fr]=v
		
		sim.simxPauseCommunication(self.clientID,True)

		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_rl,
										   rl,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_rr, 
                                            rr,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_fl, 
                                            fl,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_fr, 
                                            fr,sim.simx_opmode_oneshot)
		sim.simxPauseCommunication(self.clientID,False)

	def arm_position(self,theta):
		[theta_1,theta_2,theta_3,theta_4,theta_5]=theta

		sim.simxPauseCommunication(self.clientID,True)
		sim.simxSetJointTargetPosition(self.clientID,self.joint1,
									   theta_1,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(self.clientID,self.joint2,theta_2,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(self.clientID, 
                                        self.joint3,theta_3,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(self.clientID, 
                                       self.joint4,theta_4,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(self.clientID, 
                                       self.joint5,theta_5,sim.simx_opmode_oneshot)
		sim.simxPauseCommunication(self.clientID,False)		
	def open_grip(self,boolean):
		if(boolean==True):
			sim.simxSetJointTargetVelocity(self.clientID,self.gjoint2,-0.04,sim.simx_opmode_oneshot)
		else:
			sim.simxSetJointTargetVelocity(self.clientID,self.gjoint2,0.04,sim.simx_opmode_oneshot)
		rC,position=sim.simxGetJointPosition(self.clientID,self.gjoint2,sim.simx_opmode_blocking)
		sim.simxSetJointTargetPosition(self.clientID,self.gjoint1,position*-0.5,sim.simx_opmode_oneshot)
	def pick_up(self,objectHandle,sensorHandle,mode):
		if mode==1:#Pega o cubo do chão
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			r=sim.simxReadProximitySensor(self.clientID,sensorHandle,sim.simx_opmode_blocking)
			while(np.linalg.norm(r[2])>0.2):
				r=sim.simxReadProximitySensor(self.clientID,sensorHandle,sim.simx_opmode_blocking)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			rC,object_position = sim.simxGetObjectPosition(self.clientID,objectHandle,self.joint2,sim.simx_opmode_streaming)
			time.sleep(0.4)
			rC,object_position = sim.simxGetObjectPosition(self.clientID,objectHandle,self.joint2,sim.simx_opmode_buffer)
			rC,object_position = sim.simxGetObjectPosition(self.clientID,objectHandle,self.joint2,sim.simx_opmode_streaming)
			time.sleep(0.4)
			rC,object_position = sim.simxGetObjectPosition(self.clientID,objectHandle,self.joint2,sim.simx_opmode_buffer)
			Q=OptimizationFunction(object_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-180,[-0.13,+0.15,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.open_grip(False)
			time.sleep(1)
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)
	def put_down(self,dummyHandle,mode):
		if mode==1:#Coloca o cubo na prateleira de baixo
			rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_streaming)
			time.sleep(0.4)
			rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_buffer)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			while(dummy_position[1]>0.5):
				rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_buffer)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			Q=OptimizationFunction(dummy_position+[-0.2,-0.2,0],[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-180,[-0.13,+0.15,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			Q=OptimizationFunction(dummy_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-100,[-0.13,+0.15,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.open_grip(True)
			time.sleep(1)
			self.arm_position(np.array([0,-60,-60,-60,0])*np.pi/180)
		if mode==2:#Coloca o cubo na prateleira de cima
			rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_streaming)
			time.sleep(0.4)
			rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_buffer)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			while(dummy_position[1]>0.4):
				rC,dummy_position = sim.simxGetObjectPosition(self.clientID,dummyHandle,self.joint2,sim.simx_opmode_buffer)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			Q=OptimizationFunction(dummy_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-90,[-0.13,+0.15,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.open_grip(True)
			time.sleep(1)
			self.arm_position(np.array([0,-60,-60,-60,0])*np.pi/180)