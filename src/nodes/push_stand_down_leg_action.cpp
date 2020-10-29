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
  double ds_duration;
  double swing_leg_down_time;
  Switch help_option("h", "help", "produce help message");

  Value<double> ds_duration_option("d", "ds_duration", "ds duration option", 0., &ds_duration);
  Value<double> swing_leg_down_time_option("u", "down_time", "swing leg down time option",
                                           0., &swing_leg_down_time);


  OptionParser op("Allowed options");
  op.add(help_option);
  op.add(ds_duration_option);
  op.add(swing_leg_down_time_option);

  op.parse(argc, argv);

  // print auto-generated help message
  if (help_option.isSet() || !ds_duration_option.isSet() || !swing_leg_down_time_option.isSet())
  {
    std::cout << op << "\n";
    return 0;
  }

  // Set up ROS.
  ros::init(argc, argv, "stand_one_leg_start");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  // Wait for biped status
  pal_locomotion_msgs::BControllerStatus::ConstPtr msg;
  msg = ros::topic::waitForMessage<pal_locomotion_msgs::BControllerStatus>(
      "/biped_walking_dcm_controller/bcontroller_status", nh, ros::Duration(1.));

  if (!msg)
  {
    ROS_ERROR_STREAM("no biped controller status received");
    return 0;
  }

  Side side;
  convert(msg->side, side);
  if (side == +Side::LEFT)
  {
    side = Side::RIGHT;
  }
  else if (side == +Side::RIGHT)
  {
    side = Side::LEFT;
  }
  else
  {
    PAL_THROW_DEFAULT("side: " << side._to_string() << " not recognized");
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped transform_stamped;
  eMatrixHom relative_pose;
  try
  {
    if (side == +Side::LEFT)
    {
      transform_stamped = tf_buffer.lookupTransform("odom", "left_sole_link",
                                                    ros::Time(0), ros::Duration(1.));
      relative_pose =
          createMatrix(eQuaternion::Identity(),
                       eVector3(0., 0.0, -transform_stamped.transform.translation.z));
    }
    else
    {
      transform_stamped = tf_buffer.lookupTransform("odom", "right_sole_link",
                                                    ros::Time(0), ros::Duration(1.));
      relative_pose =
          createMatrix(eQuaternion::Identity(),
                       eVector3(0., 0.0, -transform_stamped.transform.translation.z));
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return 0;
  }

  eMatrixHom transform;
  pal::convert(transform_stamped.transform, transform);

  pal_locomotion_msgs::PushActions push_actions_request;

  pal_locomotion_msgs::ActionWithParameters new_action;
  new_action.action_type = "pal_locomotion::StandDownLegAction";

  property_bag::PropertyBag parameters;
  parameters.addProperty("side", std::string(side._to_string()));
  parameters.addProperty("ds_duration", ds_duration);

  transform = transform * relative_pose;
  eQuaternion target_swing_leg_orientation = eQuaternion(transform.rotation());
  eVector3 target_swing_leg_position = transform.translation();

  parameters.addProperty("swing_leg_duration", swing_leg_down_time);
  parameters.addProperty("target_swing_leg_orientation", target_swing_leg_orientation);
  parameters.addProperty("target_swing_leg_position", target_swing_leg_position);

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
