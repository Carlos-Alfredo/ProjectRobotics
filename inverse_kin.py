from casadi import *
from direct_kin import directf
import numpy as np
import matplotlib.pyplot as plt

def OptimizationFunction(P_desire,theta,angulo,offset):
	opti = Opti()

	#theta = [0.1552, 0.1368, 0.1917]
	q_0 =opti.variable();q_1 =opti.variable();q_2 =opti.variable();q_3 =opti.variable();


	#opti.set_initial(q_1,-30.91);opti.set_initial(q_2,-52.42);opti.set_initial(q_3,-72.68)



	f = directf(np.array([q_0,q_1,q_2,q_3]),theta)


	#P_desire = np.array([0,-2,0.3])

	opti.minimize(np.sqrt((P_desire[0]+offset[0]-f[0])**2+(P_desire[1]+offset[1]-f[1])**2+(P_desire[2]+offset[2]-f[2])**2))
	opti.subject_to(q_0<=90);opti.subject_to(-90<=q_0);
	opti.subject_to(q_1<=90);opti.subject_to(-90<=q_1);
	opti.subject_to(q_2<=90);opti.subject_to(-90<=q_2);
	opti.subject_to(q_3<=90);opti.subject_to(-90<=q_3);
	opti.subject_to(q_1+q_2+q_3==angulo);
	#opti.subject_to(q_0==-np.arctan(P_desire[2]/P_desire[1])*180/np.pi);
	opti.subject_to(q_0==0);
	opti.solver('ipopt');
	sol = opti.solve()
	return [sol.value(q_0),sol.value(q_1),sol.value(q_2),sol.value(q_3)]
