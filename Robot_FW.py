import sim
import time
import math
import numpy as np
from inverse_kin import OptimizationFunction

class Robot_FW():
	def __init__(self,clientID,RobotName):
		self.clientID = clientID
		returnCode0,self.sensorHandle_EF = sim.simxGetObjectHandle(clientID,'sensor_EF',sim.simx_opmode_blocking)
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
		
		#sim.simxPauseCommunication(self.clientID,True)

		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_rl,
										   rl,sim.simx_opmode_streaming)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_rr, 
                                            rr,sim.simx_opmode_streaming)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_fl, 
                                            fl,sim.simx_opmode_streaming)
		sim.simxSetJointTargetVelocity(self.clientID, self.eixo_fr, 
                                            fr,sim.simx_opmode_streaming)
		#sim.simxPauseCommunication(self.clientID,False)

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
			self.arm_position(np.array([0,-30,-30,-30,0])*np.pi/180)
			self.open_grip(True)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			object_position = self.get_position(objectHandle,self.joint1)
			while(object_position[1]>0.35):
				object_position = self.get_position(objectHandle,self.joint1)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			r_EF=False
			while (r_EF!=True):
				self.open_grip(True)
				self.velocity(np.array([0,0,0,0])*np.pi/180)
				object_position = self.get_position(objectHandle,self.joint1)
				if object_position[1]==0:
					object_position[1]=0.00000001
				theta=np.arctan((-object_position[2])/(object_position[1]))
				self.arm_position(np.array([theta,0,0,0,0]))
				object_position = self.get_position(objectHandle,self.joint2)
				print("Teste",object_position)
				Q=OptimizationFunction([object_position[0],np.sqrt(object_position[1]**2+object_position[2]**2),0.001],[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-180,[0,0,0])
				self.arm_position(np.array([theta*180/np.pi,Q[1],Q[2],Q[3],0])*np.pi/180)
				time.sleep(1)
				r_EF=self.prox_sensor(objectHandle,0.02)
			self.open_grip(False)
			time.sleep(1)
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)
             
		if mode==2:#Pega o cubo da prateleira de baixo
			self.arm_position(np.array([0,-60,-30,-30,0])*np.pi/180)
			self.open_grip(True)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			object_position = self.get_position(objectHandle,self.joint2)
			while(object_position[1]>0.45):
				object_position = self.get_position(objectHandle,self.joint2)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			r_EF=[0,False,0,0,0]
			while (r_EF[1]!=True):
				self.open_grip(True)
				self.velocity(np.array([0,0,0,0])*np.pi/180)
				object_position = self.get_position(objectHandle,self.joint1)
				if object_position[1]==0:
					object_position[1]=0.00000001
				theta=np.arctan((-object_position[2])/(object_position[1]))
				self.arm_position(np.array([theta*180/np.pi,-60,-30,0,0])*np.pi/180)
				time.sleep(1)
				object_position = self.get_position(objectHandle,self.joint2)
				Q=OptimizationFunction([object_position[0],np.sqrt(object_position[1]**2+object_position[2]**2),0.001],[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-150,[0,0,0])
				print("->",Q)
				self.arm_position(np.array([theta*180/np.pi,Q[1],Q[2],Q[3],0])*np.pi/180)
				time.sleep(1)
				r_EF=sim.simxReadProximitySensor(self.clientID,self.sensorHandle_EF,sim.simx_opmode_streaming)
			self.open_grip(False)
			time.sleep(1)
			self.arm_position(np.array([0,-45,-45,0,0])*np.pi/180)

		if mode==3:#Pega o cubo da prateleira de cima
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)
			self.open_grip(True)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			object_position = self.get_position(objectHandle,self.joint2)
			while(object_position[1]>0.4):
				object_position = self.get_position(objectHandle,self.joint2)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			r_EF=[0,False,0,0,0]
			while (r_EF[1]!=True):
				self.open_grip(True)
				self.velocity(np.array([0,0,0,0])*np.pi/180)
				object_position = self.get_position(objectHandle,self.joint1)
				if object_position[1]==0:
					object_position[1]=0.00000001
				theta=np.arctan((-object_position[2])/(object_position[1]))
				self.arm_position(np.array([theta,0,0,0,0]))
				object_position = self.get_position(objectHandle,self.joint2)
				Q=OptimizationFunction([object_position[0],np.sqrt(object_position[1]**2+object_position[2]**2),0.001],[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-90,[0,0,0])
				self.arm_position(np.array([theta*180/np.pi,Q[1],Q[2],Q[3],0])*np.pi/180)
				time.sleep(1)
				r_EF=sim.simxReadProximitySensor(self.clientID,self.sensorHandle_EF,sim.simx_opmode_streaming)
			self.open_grip(False)
			time.sleep(1)
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)

	def put_down(self,dummyHandle,mode,n_color):
		if mode==1:#Coloca o cubo na prateleira de baixo
			dummy_position = self.get_position(dummyHandle,self.joint2)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			while(dummy_position[1]>0.6):
				dummy_position = self.get_position(dummyHandle,self.joint2)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			print("Teste",dummy_position)
			self.arm_position(np.array([0,-30,-30,-30,0])*np.pi/180)
			time.sleep(1)
			dummy_position = self.get_position(dummyHandle,self.joint2)
			Q=OptimizationFunction(dummy_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-100,[0,0,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			dummy_position = self.get_position(dummyHandle,self.joint2)
			while(dummy_position[1]>0.45):
				dummy_position = self.get_position(dummyHandle,self.joint2)
			print("Teste",dummy_position)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			dummy_position = self.get_position(dummyHandle,self.joint2)
			Q=OptimizationFunction(dummy_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-120,[-0.05*n_color-0.02,0,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.open_grip(True)
			time.sleep(1)
			self.arm_position(np.array([0,-30,-30,-30,0])*np.pi/180)
		if mode==2:#Coloca o cubo na prateleira de cima
			dummy_position = self.get_position(dummyHandle,self.joint2)
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)
			self.velocity(np.array([-60,-60,-60,-60])*np.pi/180)
			while(dummy_position[1]>0.4):
				dummy_position = self.get_position(dummyHandle,self.joint2)
			self.velocity(np.array([0,0,0,0])*np.pi/180)
			dummy_position = self.get_position(dummyHandle,self.joint2)
			print("Teste",dummy_position)
			Q=OptimizationFunction(dummy_position,[0.15497663617134094, 0.134843647480011, 0.1936628818511963],-90,[-0.05*n_color,0,0])
			self.arm_position(np.array([Q[0],Q[1],Q[2],Q[3],0])*np.pi/180)
			time.sleep(1)
			self.open_grip(True)
			time.sleep(1)
			self.arm_position(np.array([0,0,0,0,0])*np.pi/180)
		
			
	def get_position(self,object1,object2):
		rC,p1 = sim.simxGetObjectPosition(self.clientID,object1,-1,sim.simx_opmode_streaming)
		time.sleep(0.1)
		rC,p1 = sim.simxGetObjectPosition(self.clientID,object1,-1,sim.simx_opmode_buffer)
		rC,p2 = sim.simxGetObjectPosition(self.clientID,object2,-1,sim.simx_opmode_streaming)
		time.sleep(0.1)
		rC,p2 = sim.simxGetObjectPosition(self.clientID,object2,-1,sim.simx_opmode_buffer)
		return [p2[2]-p1[2],p2[1]-p1[1],p2[0]-p1[0]]
	def prox_sensor(self,objectHandle,distancia):
		p=self.get_position(objectHandle,self.gjoint1)
		d=np.linalg.norm(p)
		return d<distancia
	def GetColors(self,clientID):
		buffer = bytearray()
		answer=sim.simxCallScriptFunction(clientID,'Floor_visible',
                                         sim.sim_scripttype_childscript,'coroutineMain',
                                         [],[],[],buffer,sim.simx_opmode_blocking)
		self.color_cube = answer[4].decode("utf-8")