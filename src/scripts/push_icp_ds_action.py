import sys
import rospy


from pal_locomotion_msgs.srv import PushActions, PushActionsRequest, PushActionsResponse
from pal_locomotion_msgs.msg import ActionWithParameters


rospy.init_node('push_icp_ds_action_node', anonymous=True)

push_action_service = rospy.ServiceProxy('/biped_walking_dcm_controller/push_actions', PushActions)

new_action = ActionWithParameters(action_type='pal_locomotion::MoveICPDSAction', action_parameters='')

request = PushActionsRequest(actions=[new_action])
print ('requst',request)
response = push_action_service(request)
print ('response', response)
