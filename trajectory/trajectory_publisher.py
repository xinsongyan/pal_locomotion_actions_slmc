import os
import rospy
import numpy as np


def publish_trajectory(filename, param_name):
    filename = os.getcwd() + '/' + filename
    trajectory = np.genfromtxt(filename, delimiter=',')
    trajectory_dict={'t':trajectory[:,0].tolist(),
                     'pos':{'x':trajectory[:,1].tolist(),
                            'y':trajectory[:,2].tolist(),
                            'z':trajectory[:,3].tolist()},
                     'vel':{'x':trajectory[:,4].tolist(),
                            'y':trajectory[:,5].tolist(),
                            'z':trajectory[:,6].tolist()},
                     'acc':{'x':trajectory[:,7].tolist(),
                            'y':trajectory[:,8].tolist(),
                            'z':trajectory[:,9].tolist()}}
    rospy.set_param('/'+param_name, trajectory_dict)

if __name__ == "__main__":
    publish_trajectory(filename='com_trajectory.csv', param_name='com_trajectory')
    publish_trajectory(filename='lfoot_trajectory.csv', param_name='lfoot_trajectory')
    publish_trajectory(filename='rfoot_trajectory.csv', param_name='rfoot_trajectory')