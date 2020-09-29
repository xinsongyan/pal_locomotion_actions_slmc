#include <dynamic_introspection/dynamic_introspection.h>
#include <pal_locomotion_actions_slmc/balance_ds_action.h>
#include <pal_locomotion/step_planner/step_planner_base.h>
#include <pal_locomotion/visualization/visualization.h>
#include <math_utils/geometry_tools.h>
#include <pal_ros_utils/conversions.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>

using namespace math_utils;

namespace pal_locomotion
{
BalanceDsAction::BalanceDsAction(ros::NodeHandle &nh,
                                 const BalanceDsActionParameters &action_parameters,
                                 BController *bController)
{
  action_parameters_ = action_parameters;
  if (!configure(nh, bController, property_bag::PropertyBag()))
  {
    PAL_THROW_DEFAULT("problem configuring actoin");
  }
}

BalanceDsAction::~BalanceDsAction()
{
}

bool BalanceDsAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &parameters)
{
  bc_ = bController;

  ROS_DEBUG_STREAM("Created double support step");

  {
    CascadeFilterParameters target_filter_params;
    FilterdParameters filter;
    filter.type_ = "LowPass";
    filter.composition_type_ = "Sequence";
    filter.order_ = 5;
    filter.cutoff1_ = action_parameters_.target_filter_cutoff_;
    filter.cutoff2_ = 0;
    target_filter_params.filter_parameters_ = { filter };

    target_filter_.reset(new math_utils::FirstOrderLowPassFilterVector3d(
        bc_->getControllerDt(), "target_filter", nh, target_filter_params));
  }

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), action_parameters_.hpl_paramters_));

  return true;
}

bool BalanceDsAction::enterHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balance double support enter hook, time: " << time.toSec());

  std::vector<Side> stanceLegIds;
  stanceLegIds.push_back(+Side::LEFT);
  stanceLegIds.push_back(+Side::RIGHT);
  std::vector<Side> swingLegIds;
  bc_->setStanceLegIDs(stanceLegIds);
  bc_->setSwingLegIDs(swingLegIds);

  bc_->setWeightDistribution(0.5);

  bc_->setActualSupportType(+SupporType::DS);

  eMatrixHom actualLeftFootPose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actualRightFootPose = bc_->getActualFootPose(+Side::RIGHT);

  initial_target_ = (actualLeftFootPose.translation() + actualRightFootPose.translation()) / 2.;

  return true;
}

/// This function is called every cycle of the loop, until the end of the
/// Action()
bool BalanceDsAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eVector3 middle_foot_point =
      (actual_left_foot_pose.translation() + actual_right_foot_pose.translation()) / 2.;

  eVector3 middle_point_filtered = target_filter_->onlineFiltering(middle_foot_point);

  eMatrixHom local_coordinate_frame =
      interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);

  double desiredYaw = extractYaw(local_coordinate_frame);

  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);

  Eigen::Vector2d referenceCOP = local_coordinate_frame_2d.translation();
  Eigen::Vector2d targetDCM = local_coordinate_frame_2d.translation();
  Eigen::Vector2d targetDCM_vel = Eigen::Vector2d::Zero();

  if (action_parameters_.icp_control_)
  {
    double desired_z = local_coordinate_frame.translation().z() + bc_->getParameters()->z_height_;
    double desired_z_d = 0.;

    eVector3 targetCOM;
    targetCOM << targetDCM.x(), targetDCM.y(), desired_z;
    eVector3 targetCOM_vel;
    targetCOM_vel << targetDCM_vel.x(), targetDCM_vel.y(), desired_z_d;

    control(bc_, rate_limiter_, targetCOM, targetCOM_vel, referenceCOP,
            action_parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
            targetCOP_unclamped_);
  }
  else
  {
    if (action_parameters_.use_filtered_target_)
    {
      bc_->setDesiredCOMPosition(
          eVector3(middle_point_filtered.x(), middle_point_filtered.y(),
                   middle_foot_point.z() + bc_->getParameters()->z_height_));
    }
    else
    {
      if (action_parameters_.use_filtered_target_)
      {
        bc_->setDesiredCOMPosition(
            eVector3(middle_point_filtered.x(), middle_point_filtered.y(),
                     middle_foot_point.z() + bc_->getParameters()->z_height_));
      }
      else
      {
        bc_->setDesiredCOMPosition(
            eVector3(middle_foot_point.x(), middle_foot_point.y(),
                     middle_foot_point.z() + bc_->getParameters()->z_height_));
      }
    }

    bc_->setDesiredCOMVelocity(eVector3::Zero());
    bc_->setDesiredCOMAcceleration(eVector3::Zero());
  }

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));

  visualize(time);

  return true;
}

/// Return "true" if the Action has to be stopped. The default implementation
/// use time to
/// stop the action;
bool BalanceDsAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

/// when isOver()=true, this function is called and the action is removed from
/// the queue.
bool BalanceDsAction::endHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balance ds end hook, time: " << time.toSec());
  return true;
}

void BalanceDsAction::visualize(const ros::Time &time)
{
}
}
