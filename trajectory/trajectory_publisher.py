import os
import rospy
import numpy as np


filename = os.getcwd() + '/com_trajectory.csv'
com_trajectory = np.genfromtxt(filename, delimiter=',')


com_trajectory_dict={'t':com_trajectory[:,0].tolist(),
                     'pos':{'x':com_trajectory[:,1].tolist(), 'y':com_trajectory[:,2].tolist(), 'z':com_trajectory[:,3].tolist()},
                     'vel':{'x':com_trajectory[:,4].tolist(), 'y':com_trajectory[:,5].tolist(), 'z':com_trajectory[:,6].tolist()},
                     'acc':{'x':com_trajectory[:,7].tolist(), 'y':com_trajectory[:,8].tolist(), 'z':com_trajectory[:,9].tolist()}}
rospy.set_param('/com_trajectory', com_trajectory_dict)


