from casadi import *
import numpy as np

def directf(Q,theta):
	q_0 = Q[0]*np.pi/180;q_1 = Q[1]*np.pi/180;q_2=Q[2]*np.pi/180;q_3=Q[3]*np.pi/180
	l1 = theta[0];l2 = theta[1];l3 = theta[2]
	Q_1 =q_1;Q_2=q_1+q_2;Q_3=q_1+q_2+q_3
	f_1 = -(l1*np.cos(Q_1)+l2*np.cos(Q_2)+l3*np.cos(Q_3));
	f_2 = -(l1*np.sin(Q_1)+l2*np.sin(Q_2)+l3*np.sin(Q_3))*np.cos(-q_0);
	f_3 = -(l1*np.sin(Q_1)+l2*np.sin(Q_2)+l3*np.sin(Q_3))*np.sin(-q_0);
	f = f_1;f = vertcat(f,f_2);f = vertcat(f,f_3);return f


def directf_return(Q,theta):
	q_1 = Q[0]*np.pi/180;q_2=Q[1]*np.pi/180;q_3=Q[2]*np.pi/180
	l1 = theta[0];l2 = theta[1];l3 = theta[2]
	f_1 = theta[3];alpha = 90-q_1-q_2;
	f_2 = l1*sin(q_1)+l2*cos(alpha)+theta[4];
	f_3 = l1*cos(q_1)+l2*sin(alpha)-l3+theta[5];
	f = f_1;f = vertcat(f,f_2);f = vertcat(f,f_3);
	return f
