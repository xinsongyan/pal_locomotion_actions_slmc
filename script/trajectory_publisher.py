import os
import rospy
import numpy as np

def publish_footplacements(footplacements, param_name):
    dict={'data':footplacements.flatten().tolist(),
                     'row_number': footplacements.shape[0],
                     'col_number': footplacements.shape[1]}
    rospy.set_param('/'+param_name, dict)
    # print('data name is ', footplacements)

def publish_com_trajectory(trajectory, param_name):
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

def publish_zmp_trajectory(trajectory, param_name): # not used
    trajectory_dict={'t':trajectory[:,0].tolist(),
                     'x':trajectory[:,1].tolist(),
                     'y':trajectory[:,2].tolist(),
                     'z':trajectory[:,3].tolist()}
    rospy.set_param('/'+param_name, trajectory_dict)

def publish_support_durations(support_durations, param_name):
    rospy.set_param('/'+param_name, support_durations.tolist())

def publish_support_end_times(support_durations, param_name):
    support_end_times = np.cumsum(support_durations)
    rospy.set_param('/'+param_name, support_end_times.tolist())

def publish_support_indexes(support_indexes, param_name):
    rospy.set_param('/'+param_name, support_indexes.tolist())

def publish_swing_height(param_name, swing_height):
    rospy.set_param('/'+param_name, swing_height)

def publish_gains(param_name, com_gain):
    rospy.set_param('/'+param_name+'/com_fb_kp', com_gain[0])
    rospy.set_param('/'+param_name+'/com_fb_kd', com_gain[1])
    rospy.set_param('/'+param_name+'/com_ff_kp', com_gain[2])


def publish_all(com_trajectory, support_durations, support_indexes, foot_placements, swing_height, com_gain):
    try:
        while not rospy.is_shutdown():
            publish_com_trajectory(trajectory=com_trajectory, param_name='com_trajectory')
            publish_support_durations(support_durations=support_durations, param_name='support_durations')
            publish_support_end_times(support_durations=support_durations, param_name='support_end_times')
            publish_support_indexes(support_indexes=support_indexes, param_name='support_indexes')
            publish_footplacements(footplacements=foot_placements, param_name='foot_placements')
            publish_swing_height(swing_height=swing_height, param_name='swing_height')
            publish_gains(com_gain=com_gain, param_name='com_gain')
            break
    except rospy.ROSInterruptException: pass


if __name__ == "__main__":
    publish_all()