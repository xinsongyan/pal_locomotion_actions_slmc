import time
import rospy
import roslaunch

from trajectory.dsp_test3.trajectory_publisher import publish_all


def launch_roscore():
    import subprocess
    roscore_process = subprocess.Popen('roscore')




def launch_talos_and_dcm_controller2():
    rospy.init_node('talos_locomotion', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args1 = ['/opt/pal/ferrum/share/talos_gazebo/launch/talos_gazebo.launch','vel:=2.19']
    cli_args2 = ['/home/xin/catkin_ws/src/talos_pal_locomotion/launch/talos_dcm_walking_controller.launch','estimator:=kinematic_estimator_params']

    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    roslaunch_args1 = cli_args1[1:]

    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
    roslaunch_args2 = cli_args2[1:]

    launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    parent.start()




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



def main():
    launch_roscore()
    launch_talos_and_dcm_controller2()
    publish_all()
    push_csv_walking_action_prev()
    while True:
        pass



if __name__ == "__main__":
    main()

