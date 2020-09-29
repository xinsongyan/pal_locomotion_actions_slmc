#include <ros/ros.h>
#include <pal_locomotion_msgs/ActionWithParameters.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <property_bag/property_bag.h>
#include <pal_locomotion/step.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <tf/transform_listener.h>
#include <pal_utils/popl.hpp>
#include <pal_locomotion_msgs/BControllerStatus.h>
#include <pal_locomotion/conversions.h>
#include <pal_ros_utils/conversions.h>

using namespace popl;
using namespace pal_locomotion;

int main(int argc, char **argv)
{ 
  // Set up args.
  double duration = 3.0;
  double x = 0.0;
  double y = 0.1;
  double z = 0.0;


  Switch help_option("h", "help", "produce help message");

  Value<double> duration_option("d", "duration", "duration option", 0., &duration);
  Value<double> target_com_x_option("x", "com_x", "target com x option", 0., &x);
  Value<double> target_com_y_option("y", "com_y", "target com y option", 0., &y);
  Value<double> target_com_z_option("z", "com_z", "target com z option", 0., &z);

  OptionParser op("Allowed options");
  op.add(help_option);
  op.add(duration_option);
  op.add(target_com_x_option);
  op.add(target_com_y_option);
  op.add(target_com_z_option);

  op.parse(argc, argv);

  if (help_option.isSet() || !duration_option.isSet() || !target_com_x_option.isSet() || !target_com_y_option.isSet() || !target_com_z_option.isSet())
  {
    std::cout << op << "\n";
    return 0;
  }

  // Set up ROS.
  ros::init(argc, argv, "move_com_action_start");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  pal_locomotion_msgs::PushActions push_actions_request;

  pal_locomotion_msgs::ActionWithParameters new_action;
  new_action.action_type = "pal_locomotion::COMAction";
  property_bag::PropertyBag parameters;
  parameters.addProperty("target_com_position", eVector3(x, y, z));
  parameters.addProperty("duration", duration);

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << parameters;
  new_action.action_parameters = ss.str();
  push_actions_request.request.actions.push_back(new_action);

  if (client.call(push_actions_request))
  {
    ROS_INFO("Succesfully called service");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
}