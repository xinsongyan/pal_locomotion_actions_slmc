import sys
import rospy


from pal_locomotion_msgs.srv import PushActions, PushActionsRequest
from pal_locomotion_msgs.msg import ActionWithParameters



rospy.init_node('push_csv_com_action_node', anonymous=True)

rospy.wait_for_service('/biped_walking_dcm_controller/push_actions')

push_action_service = rospy.ServiceProxy('/biped_walking_dcm_controller/push_actions', PushActions)

new_action = ActionWithParameters(action_type='pal_locomotion::CSVCOMAction')

request = PushActionsRequest(actions=[new_action])
print ('requst',request)
response = push_action_service(request)
print ('response', response)
