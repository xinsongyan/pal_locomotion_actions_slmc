#include <ros/ros.h>
#include <pal_locomotion_msgs/ActionWithParameters.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <property_bag/property_bag.h>
#include <pal_locomotion/step.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <pal_utils/popl.hpp>
#include <tf/transform_listener.h>
#include <pal_ros_utils/conversions.h>

using namespace popl;
using namespace pal_locomotion;

int main(int argc, char** argv)
{
  Side side;
  double ds_duration;
  double swing_leg_up_time;
  double swing_leg_height;

  Switch help_option("h", "help", "produce help message");
  Value<Side> side_option("s", "side", "side of the stance leg", Side::LEFT, &side);
  Value<double> ds_duration_option("d", "ds_duration", "ds duration option", 0., &ds_duration);
  Value<double> swing_leg_up_time_option("u", "up_time", "swing leg up time option", 0., &swing_leg_up_time);
  Value<double> swing_leg_height_option("z", "swing_height", "swing leg up height option",0., &swing_leg_height);


  OptionParser op("Allowed options");
  op.add(help_option);
  op.add(side_option);
  op.add(ds_duration_option);
  op.add(swing_leg_up_time_option);
  op.add(swing_leg_height_option);

  op.parse(argc, argv);

  // print auto-generated help message
  if (help_option.isSet() || !side_option.isSet() || !ds_duration_option.isSet() ||
      !swing_leg_up_time_option.isSet() || !swing_leg_height_option.isSet())
  {
    std::cout << op << "\n";
    return 0;
  }

  // Set up ROS.
  ros::init(argc, argv, "stand_one_leg_start");
  ros::NodeHandle nh;

  //  tf2_ros::Buffer tf_buffer;
  //  tf2_ros::TransformListener tf_listener(tf_buffer);

  tf::TransformListener listener;

  ros::Duration(2.).sleep();

  //  geometry_msgs::TransformStamped transform_stamped;
  //  eMatrixHom relative_pose;
  //  try
  //  {
  //    if (side == +Side::LEFT)
  //    {
  //      transform_stamped = tf_buffer.lookupTransform("odom", "right_sole_link",
  //                                                    ros::Time(0), ros::Duration(1.));
  //      relative_pose =
  //          createMatrix(eQuaternion::Identity(), eVector3(0., 0.0, swing_leg_height));
  //    }
  //    else
  //    {
  //      transform_stamped = tf_buffer.lookupTransform("odom", "left_sole_link",
  //                                                    ros::Time(0), ros::Duration(1.));
  //      relative_pose =
  //          createMatrix(eQuaternion::Identity(), eVector3(0., 0.0, swing_leg_height));
  //    }
  //  }
  //  catch (tf2::TransformException &ex)
  //  {
  //    ROS_WARN("%s", ex.what());
  //    ros::Duration(1.0).sleep();
  //    return 0;
  //  }

  tf::StampedTransform transform_stamped;
  eMatrixHom relative_pose;
  try
  {
    if (side == +Side::LEFT)
    {
      listener.lookupTransform("/odom", "/right_sole_link", ros::Time(0), transform_stamped);
      relative_pose = createMatrix(eQuaternion::Identity(), eVector3(0., 0.0, swing_leg_height));
    }
    else
    {
      listener.lookupTransform("/odom", "/left_sole_link", ros::Time(0), transform_stamped);
      relative_pose = createMatrix(eQuaternion::Identity(), eVector3(0., 0.0, swing_leg_height));
    }
  }
  catch (tf2::TransformException& ex)
  {
    PAL_THROW_DEFAULT(ex.what());
    ros::Duration(1.0).sleep();
  }

  // Create swing leg target
  eMatrixHom transform;
  pal::convert(transform_stamped, transform);
  std::cerr << "Transform: " << std::endl << transform.matrix() << std::endl;

  transform = transform * relative_pose;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  pal_locomotion_msgs::PushActions push_actions_request;

  pal_locomotion_msgs::ActionWithParameters new_action;
  new_action.action_type = "pal_locomotion::SStandOneLegAction";

  property_bag::PropertyBag parameters;
  parameters.addProperty("side", std::string(side._to_string()));
  parameters.addProperty("ds_duration", ds_duration);

  std::vector<double> swing_leg_duration = { swing_leg_up_time };
  std::vector<eQuaternion> target_swing_leg_orientation = { eQuaternion(transform.rotation()) };
  std::vector<eVector3> target_swing_leg_position = { transform.translation() };

  parameters.addProperty("swing_leg_duration", swing_leg_duration);
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
