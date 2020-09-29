#include <ros/ros.h>
#include <pal_locomotion_msgs/ActionWithParameters.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <property_bag/property_bag.h>
#include <pal_locomotion/step.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <property_bag/serialization/property_bag_boost_serialization.h>

using namespace pal_locomotion;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "wbc_action_start");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  pal_locomotion_msgs::PushActions pushActionsRequest;

  pal_locomotion_msgs::ActionWithParameters newAction;
  newAction.action_type = "pal_locomotion::WBCAction";
  property_bag::PropertyBag parameters;
  parameters.addProperty("stack_configuration", std::string("talos_stack_both_hands_head"));
  parameters.addProperty("solver_type",
                         std::string("QpReductionuEqualitiesQuadprogHeapAllocation"));
  parameters.addProperty("wbc_dt", 0.008);

  parameters.addProperty("cartesian_position_task_weight", 0.3);
  parameters.addProperty("cartesian_orientation_task_weight", 0.3);


  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << parameters;
  newAction.action_parameters = ss.str();
  pushActionsRequest.request.actions.push_back(newAction);

  if (client.call(pushActionsRequest))
  {
    ROS_INFO("Succesfully called service");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
}
