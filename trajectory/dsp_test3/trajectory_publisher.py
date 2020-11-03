import os
import rospy
import numpy as np


def publish_trajectory(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    trajectory = np.genfromtxt(filename_, delimiter=',')
    trajectory_dict={'t':trajectory[:,0].tolist(),
                     'pos':{'x':trajectory[:,1].tolist(),
                            'y':trajectory[:,2].tolist(),
                            'z':trajectory[:,3].tolist()},
                     'vel':{'x':trajectory[:,4].tolist(),
                            'y':trajectory[:,5].tolist(),
                            'z':trajectory[:,6].tolist()},
                     'acc':{'x':np.zeros_like(trajectory[:,4]).tolist(),
                            'y':np.zeros_like(trajectory[:,5]).tolist(),
                            'z':np.zeros_like(trajectory[:,6]).tolist()}}
    rospy.set_param('/'+param_name, trajectory_dict)

def publish_zmp_trajectory(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    trajectory = np.genfromtxt(filename_, delimiter=',')
    trajectory_dict={'t':trajectory[:,0].tolist(),
                     'x':trajectory[:,1].tolist(),
                     'y':trajectory[:,2].tolist(),
                     'z':trajectory[:,3].tolist()}
    rospy.set_param('/'+param_name, trajectory_dict)

def publish_support_durations(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    support_durations = np.genfromtxt(filename_, delimiter=',')
    rospy.set_param('/'+param_name, support_durations.tolist())

def publish_support_end_times(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    support_durations = np.genfromtxt(filename_, delimiter=',')
    support_end_times = np.cumsum(support_durations)
    rospy.set_param('/'+param_name, support_end_times.tolist())

def publish_support_indexes(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    support_indexes = np.genfromtxt(filename_, delimiter=',')
    rospy.set_param('/'+param_name, support_indexes.tolist())



def publish_all():
    try:
        while not rospy.is_shutdown():
            publish_trajectory(filename='com_traj(fast).csv', param_name='com_trajectory')
            # publish_zmp_trajectory(filename='zmp_trajectory.csv', param_name='zmp_trajectory')
            publish_support_durations(filename='support_durations.csv', param_name='support_durations')
            publish_support_end_times(filename='support_durations.csv', param_name='support_end_times')
            publish_support_indexes(filename='support_indexes.csv', param_name='support_indexes')

            break
    except rospy.ROSInterruptException: pass

if __name__ == "__main__":
    publish_all()