import time
import rospy
import roslaunch

from trajectory.dsp_test2.trajectory_publisher import publish_all


def launch_roscore():
    import subprocess
    roscore_process = subprocess.Popen('roscore')
    # time.sleep(1)  # wait a bit to be sure the roscore is really launched


def launch_talos_gazebo():
    rospy.init_node('node1', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = ['/opt/pal/ferrum/share/talos_gazebo/launch/talos_gazebo.launch','vel:=2.19']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent1 = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent1.start()
    # time.sleep(3)  # wait a bit


def launch_talos_dcm_walking_controller():
    # rospy.init_node('node2', anonymous=True)
    uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args2 = ['/opt/pal/ferrum/share/talos_pal_locomotion/launch/talos_dcm_walking_controller.launch','estimator:=kinematic_estimator_params']
    roslaunch_args2 = cli_args2[1:]
    roslaunch_file2 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0], roslaunch_args2)]
    parent2 = roslaunch.parent.ROSLaunchParent(uuid2, roslaunch_file2)
    parent2.start()



def push_trajectory_to_ros_param_server():
    import numpy as np
    from os.path import expanduser
    home = expanduser("~")

    # filename = home + '/catkin_ws/src/pal_locomotion_actions_slmc/trajectory/com_trajectory.csv'
    filename = home + '/catkin_ws/src/pal_locomotion_actions_slmc/trajectory/dsp_test2/com_trajectory.csv'
    com_trajectory = np.genfromtxt(filename, delimiter=',')
    com_trajectory_dict={'t':com_trajectory[:,0].tolist(),
                         'pos':{'x':com_trajectory[:,1].tolist(), 'y':com_trajectory[:,2].tolist(), 'z':com_trajectory[:,3].tolist()},
                         'vel':{'x':com_trajectory[:,4].tolist(), 'y':com_trajectory[:,5].tolist(), 'z':com_trajectory[:,6].tolist()},
                         'acc':{'x':com_trajectory[:,7].tolist(), 'y':com_trajectory[:,8].tolist(), 'z':com_trajectory[:,9].tolist()}}
    rospy.set_param('/com_trajectory', com_trajectory_dict)





def push_csv_walking_action_prev():

    from pal_locomotion_msgs.srv import PushActions, PushActionsRequest
    from pal_locomotion_msgs.msg import ActionWithParameters

    # rospy.init_node('push_csv_com_action_node', anonymous=True)
    rospy.sleep(3)
    rospy.wait_for_service('/biped_walking_dcm_controller/push_actions')

    push_action_service = rospy.ServiceProxy('/biped_walking_dcm_controller/push_actions', PushActions)

    new_action = ActionWithParameters(action_type='pal_locomotion::CSVWALKINGActionPrev')

    request = PushActionsRequest(actions=[new_action])
    print ('requst',request)
    response = push_action_service(request)
    print ('response', response)

def pause(time_in_sec):
    import time
    time.sleep(time_in_sec)


def main():
    launch_roscore()
    launch_talos_gazebo()
    launch_talos_dcm_walking_controller()
    publish_all()
    push_csv_walking_action_prev()
    while True:
        pass

if __name__ == "__main__":
    main()

