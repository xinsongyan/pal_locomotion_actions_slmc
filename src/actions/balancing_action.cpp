#include <pal_locomotion_actions_slmc/balancing_action.h>
#include <pal_locomotion/step_planner/step_planner_base.h>
#include <pal_locomotion/visualization/visualization.h>
#include <math_utils/geometry_tools.h>
#include <math_utils/eigen/rotation.h>
#include <pal_ros_utils/conversions.h>

namespace pal_locomotion
{
BalancingAction::BalancingAction(BController *bc)
  : reference_loader_(new pal_robot_tools::PluginlibHelper<pal_robot_tools::ReferenceAbstract>(
        "pal_ros_utils", "pal_robot_tools::ReferenceAbstract"))
{
  ROS_DEBUG_STREAM("Created balancing action");

  bc_ = bc;

  parameters_.readConfig<ariles::ros>(bc_->getNodeHandle(), "biped_parameters/balancing_action_parameters");

  if (true == parameters_.one_leg_support_.empty())
  {
    stage_ = STAGE_FOLLOW_REFERENCE_INITIALIZE;
  }
  else
  {
    stage_ = STAGE_MOVE_COM_INITIALIZE;
  }
  reference_ = reference_loader_->load(parameters_.reference_type_);
}

BalancingAction::~BalancingAction()
{
}

bool BalancingAction::configure(ros::NodeHandle & /*nh*/, BController *bc,
                                const property_bag::PropertyBag & /*parameters*/)
{
  ROS_DEBUG_STREAM("Configured balancing action");
  ROS_INFO("Property bag parameters are ignored in balancing action.");

  bc_ = bc;

  parameters_.readConfig<ariles::ros>(bc_->getNodeHandle(), "biped_parameters/balancing_action_parameters");

  if (true == parameters_.one_leg_support_.empty())
  {
    stage_ = STAGE_FOLLOW_REFERENCE_INITIALIZE;
  }
  else
  {
    stage_ = STAGE_MOVE_COM_INITIALIZE;
  }
  // AS for some reason this is necessary
  reference_loader_ =
      boost::make_shared<pal_robot_tools::PluginlibHelper<pal_robot_tools::ReferenceAbstract> >(
          "pal_ros_utils", "pal_robot_tools::ReferenceAbstract");
  reference_ = reference_loader_->load(parameters_.reference_type_);
  return true;
}


bool BalancingAction::enterHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balancing action enter hook, time: " << time.toSec());

  std::vector<Side> stanceLegIds;
  stanceLegIds.push_back(+Side::LEFT);
  stanceLegIds.push_back(+Side::RIGHT);
  std::vector<Side> swingLegIds;
  bc_->setStanceLegIDs(stanceLegIds);
  bc_->setSwingLegIDs(swingLegIds);

  bc_->setWeightDistribution(0.5);

  bc_->setActualSupportType(+SupporType::DS);

  return true;
}

/// This function is called every cycle of the loop, until the end of the
/// Action()
bool BalancingAction::cycleHook(const ros::Time &time)
{
  if (true == parameters_.one_leg_support_.empty())
  {
    Eigen::Isometry3d actual_foot_left_pose = bc_->getActualFootPose(+Side::LEFT);
    Eigen::Isometry3d actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
    local_coordinate_frame_ =
        interpolateBetweenTransforms(actual_foot_left_pose, actual_right_foot_pose);
  }
  else
  {
    local_coordinate_frame_ = bc_->getActualAnklePose(parameters_.support_leg_);
  }
  local_coordinate_frame_.linear() =
      pal::math_utils::matrixRollPitchYaw(0., 0., extractYaw(local_coordinate_frame_));



  switch (stage_)
  {
    case STAGE_MOVE_COM_INITIALIZE:
    {
      initial_pose_.translation() = bc_->getActualCOMPosition();
      initial_pose_.linear() =
          pal::math_utils::matrixRollPitchYaw(0., 0., extractYaw(local_coordinate_frame_));

      initial_pose_ = local_coordinate_frame_.inverse() * initial_pose_;


      Eigen::Isometry3d desired_pose = initial_pose_;
      desired_pose.translation().x() = 0.0;
      desired_pose.translation().y() = 0.0;

      stage_timer_ = ros::Duration(0.0);
      stage_duration_ = ros::Duration(10.0);
      com_trajectory_.initialize(stage_duration_.toSec(), initial_pose_.translation(),
                                 desired_pose.translation());
      com_position_sample_.setZero(3);

      stage_timer_ += bc_->getControllerDt();
    }
      stage_ = STAGE_MOVE_COM;
    // no break here

    case STAGE_MOVE_COM:
      com_trajectory_.sample(stage_timer_.toSec(), com_position_sample_);

      bc_->setDesiredCOMPosition(local_coordinate_frame_.rotation() * com_position_sample_.position_ +
                                 local_coordinate_frame_.translation());
      bc_->setDesiredCOMVelocity(local_coordinate_frame_.rotation() * com_position_sample_.velocity_);
      bc_->setDesiredCOMAcceleration(local_coordinate_frame_.rotation() *
                                     com_position_sample_.acceleration_);

      if (+Side::LEFT == parameters_.support_leg_)
      {
        bc_->setWeightDistribution(0.5 + 0.5 * (stage_timer_.toSec() / stage_duration_.toSec()));
      }
      else
      {
        bc_->setWeightDistribution(0.5 - 0.5 * (stage_timer_.toSec() / stage_duration_.toSec()));
      }

      bc_->setDesiredBaseOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));
      bc_->setDesiredTorsoOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));

      stage_timer_ += bc_->getControllerDt();
      if (stage_timer_ >= stage_duration_)
      {
        stage_ = STAGE_LIFT_LEG_INITIALIZE;
      }
      break;

    // -----

    case STAGE_LIFT_LEG_INITIALIZE:
    {
      initial_foot_pose_ = bc_->getActualFootPose(parameters_.swing_leg_);
      desired_foot_pose_ = initial_foot_pose_;
      desired_foot_pose_.translation() +=
          desired_foot_pose_.rotation() * parameters_.swing_leg_relative_position_;

      initial_foot_pose_ = local_coordinate_frame_.inverse() * initial_foot_pose_;
      desired_foot_pose_ = local_coordinate_frame_.inverse() * desired_foot_pose_;


      stage_timer_ = ros::Duration(0.0);
      stage_duration_ = ros::Duration(10.0);
      foot_trajectory_.initialize(stage_duration_.toSec(), initial_foot_pose_.translation(),
                                  desired_foot_pose_.translation());
      foot_position_sample_.setZero(3);

      stage_timer_ += bc_->getControllerDt();

      std::vector<Side> stanceLegIds;
      stanceLegIds.push_back(parameters_.support_leg_);
      std::vector<Side> swingLegIds;
      swingLegIds.push_back(parameters_.swing_leg_);
      bc_->setStanceLegIDs(stanceLegIds);
      bc_->setSwingLegIDs(swingLegIds);

      bc_->setActualSupportType(+SupporType::SS);
    }
      stage_ = STAGE_LIFT_LEG;
    // no break here

    case STAGE_LIFT_LEG:
      bc_->setDesiredCOMPosition(local_coordinate_frame_.rotation() * com_position_sample_.position_ +
                                 local_coordinate_frame_.translation());
      bc_->setDesiredCOMVelocity(eVector3(0., 0., 0.));
      bc_->setDesiredCOMAcceleration(eVector3(0., 0., 0.));

      foot_trajectory_.sample(stage_timer_.toSec(), foot_position_sample_);

      desired_foot_pose_ = local_coordinate_frame_.rotation() * initial_foot_pose_;
      desired_foot_pose_.translation() =
          local_coordinate_frame_.rotation() * foot_position_sample_.position_ +
          local_coordinate_frame_.translation();
      bc_->setDesiredFootState(
          static_cast<int>(parameters_.swing_leg_), desired_foot_pose_,
          local_coordinate_frame_.rotation() * foot_position_sample_.velocity_,
          local_coordinate_frame_.rotation() * foot_position_sample_.acceleration_,
          eVector3(0., 0., 0.), eVector3(0., 0., 0.));

      bc_->setDesiredBaseOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));
      bc_->setDesiredTorsoOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));

      stage_timer_ += bc_->getControllerDt();

      if (stage_timer_ >= stage_duration_)
      {
        stage_ = STAGE_FOLLOW_REFERENCE_INITIALIZE;
      }
      break;

    // -----

    case STAGE_FOLLOW_REFERENCE_INITIALIZE:
    {
      initial_pose_.translation() = bc_->getActualCOMPosition();
      initial_pose_.linear() =
          pal::math_utils::matrixRollPitchYaw(0., 0., extractYaw(local_coordinate_frame_));

      initial_pose_ = local_coordinate_frame_.inverse() * initial_pose_;

      bool status = false;
      if (true == parameters_.one_leg_support_.empty())
      {
        status = reference_->configure(bc_->getNodeHandle(), bc_->getControllerDt(), "com_position",
                                       "ankle_midpoint", "ankle_midpoint", initial_pose_);
      }
      else
      {
        status = reference_->configure(
            bc_->getNodeHandle(), bc_->getControllerDt(), "com_position",
            std::string("ankle_") + parameters_.one_leg_support_,
            std::string("ankle_") + parameters_.one_leg_support_, initial_pose_);
      }

      if (false == status)
      {
        ROS_ERROR("Couldn't configure reference");
        return false;
      }
    }
      stage_ = STAGE_FOLLOW_REFERENCE;
    // no break here

    case STAGE_FOLLOW_REFERENCE:
    {
      reference_->integrate(time, bc_->getControllerDt());

      bc_->setDesiredCOMPosition(local_coordinate_frame_.rotation() *
                                     reference_->getDesiredPose().translation() +
                                 local_coordinate_frame_.translation());
      bc_->setDesiredCOMVelocity(local_coordinate_frame_.rotation() *
                                 reference_->getDesiredVelocity().first);
      bc_->setDesiredCOMAcceleration(local_coordinate_frame_.rotation() *
                                     reference_->getDesiredAcceleration().first);


      bc_->setDesiredBaseOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));
      bc_->setDesiredTorsoOrientation(Eigen::Quaterniond(local_coordinate_frame_.linear()));
      /*
      bc_->setDesiredTorsoOrientation(Eigen::Quaterniond(reference_->getDesiredPose().linear()));
      Eigen::Vector3d desired_angular_vel = reference_->getDesiredVelocity().second;
      Eigen::Vector3d desired_angular_acc = reference_->getDesiredAcceleration().second;
      */

      if (false == parameters_.one_leg_support_.empty())
      {
        desired_foot_pose_ = local_coordinate_frame_.rotation() * initial_foot_pose_;
        desired_foot_pose_.translation() =
            local_coordinate_frame_.rotation() * foot_position_sample_.position_ +
            local_coordinate_frame_.translation();

        bc_->setDesiredFootState(static_cast<int>(parameters_.swing_leg_), desired_foot_pose_,
                                 eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.), eVector3(0., 0., 0.));
      }
    }
    break;

    // -----

    default:
      PAL_THROW("Internal logic error.");
  }

  bc_->setDesiredICP(eVector3(0., 0., 0.));
  bc_->setDesiredCOPReference(eVector3(0., 0., 0.));
  bc_->setDesiredCOPComputed(eVector3(0., 0., 0.));
  bc_->setDesiredComputedCMP(eVector3(0., 0., 0.));
  bc_->setActualICP(eVector3(0., 0., 0.));

  return true;
}

/// Never over
bool BalancingAction::isOverHook(const ros::Time &time)
{
  return false;
}

/// when isOver()=true, this function is called and the action is removed from
/// the queue.
bool BalancingAction::endHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balancing action end hook, time: " << time.toSec());
  return true;
}
}
