import os
import rospy
import numpy as np

def publish_footPlacement_trajectory_2d(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    trajectory = np.genfromtxt(filename_, delimiter=',')
    trajectory_dict={'data':trajectory.flatten().tolist(),
                     'row_number': trajectory.shape[0],
                     'col_number': trajectory.shape[1]}
    rospy.set_param('/'+param_name, trajectory_dict)
    print('dafa name is ', trajectory)

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
                     'acc':{'x':trajectory[:,7].tolist(),
                            'y':trajectory[:,8].tolist(),
                            'z':trajectory[:,9].tolist()}}
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

def publish_footPlacement_indexes(filename, param_name):
    filename_ = os.getcwd() + '/' + filename
    support_indexes = np.genfromtxt(filename_, delimiter=',')
    rospy.set_param('/'+param_name, support_indexes.tolist())


def publish_all():
    try:
        while not rospy.is_shutdown():
            publish_trajectory(filename='com_trajectory.csv', param_name='com_trajectory')
            publish_zmp_trajectory(filename='zmp_trajectory.csv', param_name='zmp_trajectory')
            publish_support_durations(filename='support_durations.csv', param_name='support_durations')
            publish_support_end_times(filename='support_durations.csv', param_name='support_end_times')
            publish_support_indexes(filename='support_indexes.csv', param_name='support_indexes')
            publish_footPlacement_trajectory_2d(filename='foot_placement.csv', param_name='foot_placements')
            break
    except rospy.ROSInterruptException: pass

if __name__ == "__main__":
    publish_all()