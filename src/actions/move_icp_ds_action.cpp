#include <pal_locomotion_actions/move_icp_ds_action.h>
#include <pal_locomotion_actions/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;

namespace pal_locomotion
{
MoveICPDSAction::MoveICPDSAction()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{
}

MoveICPDSAction::~MoveICPDSAction()
{
}

bool MoveICPDSAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &parameters)
{
  bc_ = bController;
  switch_time_ = internal_time_ + ros::Duration(parameters_.switch_duration_);

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  icp_interpolator_.reset(new MinJerkGenerator2D());

  ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(ros::NodeHandle(nh, "move_icp_ds")));
  ddr_->RegisterVariable(&parameters_.switch_duration_, "switch_duration", 0., 10.);
  ddr_->PublishServicesTopics();

  return true;
}

bool MoveICPDSAction::enterHook(const ros::Time &time)
{
  support_type_ = BalanceSupporType::LEFT;

  return true;
}

bool MoveICPDSAction::cycleHook(const ros::Time &time)
{
  switch_time_ = ros::Time(0) + ros::Duration(parameters_.switch_duration_);

  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame;

  if (support_type_ == BalanceSupporType::DS)
  {
    local_coordinate_frame =
        interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
  }
  else if (support_type_ == BalanceSupporType::LEFT)
  {
    local_coordinate_frame = actual_left_foot_pose;
  }
  else if (support_type_ == BalanceSupporType::RIGHT)
  {
    local_coordinate_frame = actual_right_foot_pose;
  }

  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);

  double desiredYaw =
      extractYaw(interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose));

  if (configure_interpolator_)
  {
    if (initial_interpolation_)
    {
      icp_interpolator_->initialize(
          { internal_time_, switch_time_ },
          { (local_coordinate_frame.inverse() * interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose)).translation().segment(0, 2), eVector2::Zero()},
          { eVector2::Zero(), eVector2::Zero() },
          { eVector2::Zero(), eVector2::Zero() });

      initial_interpolation_ = false;
    }
    else
    {
      if (support_type_ == BalanceSupporType::LEFT)
      {
        icp_interpolator_->initialize(
            { internal_time_, switch_time_ },
            { (local_coordinate_frame.inverse() * actual_right_foot_pose).translation().segment(0, 2),
              eVector2::Zero() },
            { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(), eVector2::Zero() });
      }
      else if (support_type_ == BalanceSupporType::RIGHT)
      {
        icp_interpolator_->initialize(
            { internal_time_, switch_time_ },
            { (local_coordinate_frame.inverse() * actual_left_foot_pose).translation().segment(0, 2),
              eVector2::Zero() },
            { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(), eVector2::Zero() });
      }
      else
      {
        PAL_THROW_DEFAULT("not supported");
      }
    }

    configure_interpolator_ = false;
  }

  eVector2 local_target_dcm;
  eVector2 local_target_dcm_vel;
  eVector2 local_target_dcm_acc;
  icp_interpolator_->query(internal_time_, local_target_dcm, local_target_dcm_vel,
                           local_target_dcm_acc);

  eVector2 global_target_dcm = local_coordinate_frame_2d * local_target_dcm;
  eVector2 global_target_cop = global_target_dcm;
  eVector2 global_target_dcm_vel = local_coordinate_frame_2d.rotation() * local_target_dcm_vel;

  double desired_z = local_coordinate_frame.translation().z() + bc_->getParameters()->z_height_;
  double desired_z_d = 0.;

  eVector3 targetCOM;
  targetCOM << global_target_dcm.x(), global_target_dcm.y(), desired_z;
  eVector3 targetCOM_vel;
  targetCOM_vel << global_target_dcm_vel.x(), global_target_dcm_vel.y(), desired_z_d;

  control(bc_, rate_limiter_, targetCOM, targetCOM_vel, global_target_cop,
          parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
          targetCOP_unclamped_);

  internal_time_ += bc_->getControllerDt();

  if (internal_time_ > switch_time_)
  {
    internal_time_ = ros::Time(0.);
    if (support_type_ == BalanceSupporType::DS)
    {
      // support_type_ = LEFT_SS;
    }
    else if (support_type_ == BalanceSupporType::LEFT)
    {
      ROS_INFO_STREAM("moving ICP to right foot");
      support_type_ = BalanceSupporType::RIGHT;
    }
    else if (support_type_ == BalanceSupporType::RIGHT)
    {
      ROS_INFO_STREAM("moving ICP to left foot");
      support_type_ = BalanceSupporType::LEFT;
    }
    configure_interpolator_ = true;
  }

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));

  return true;
}

bool MoveICPDSAction::isOverHook(const ros::Time &time)
{
  return false;
}

bool MoveICPDSAction::endHook(const ros::Time &time)
{
  return true;
}
}
