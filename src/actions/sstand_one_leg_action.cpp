#include <pal_locomotion_actions_slmc/sstand_one_leg_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace pal_robot_tools;
using namespace math_utils;

namespace pal_locomotion
{
SStandOneLegAction::SStandOneLegAction()
  : internal_time_(ros::Time(0)), swing_leg_configured_(false), swing_leg_pose_index_(0)
{
}

bool SStandOneLegAction::configure(ros::NodeHandle &nh, BController *bc,
                                  const property_bag::PropertyBag &pb)
{
  bc_ = bc;

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  icp_interpolator_.reset(new MinJerkGenerator2D());

  force_distribution_interpolator_.reset(new MinJerkGenerator());

  pb.getPropertyValue<double>("ds_duration", parameters_.ds_duration_,
                              property_bag::RetrievalHandling::THROW);

  pb.getPropertyValue<std::vector<double>>("swing_leg_duration", parameters_.swing_leg_duration_,
                                           property_bag::RetrievalHandling::THROW);
  pb.getPropertyValue<std::vector<eQuaternion>>("target_swing_leg_orientation",
                                                parameters_.target_swing_leg_orientation_,
                                                property_bag::RetrievalHandling::THROW);

  pb.getPropertyValue<std::vector<eVector3>>("target_swing_leg_position",
                                             parameters_.target_swing_leg_position_,
                                             property_bag::RetrievalHandling::THROW);

  std::string side_str;
  pb.getPropertyValue<std::string>("side", side_str, property_bag::RetrievalHandling::THROW);
  parameters_.side_ = Side::_from_string(side_str.c_str());

  PAL_ASSERT_PERSIST_BIGGER(parameters_.swing_leg_duration_.size(), 0);
  PAL_ASSERT_PERSIST_EQUAL(parameters_.swing_leg_duration_.size(),
                           parameters_.target_swing_leg_position_.size());
  PAL_ASSERT_PERSIST_EQUAL(parameters_.swing_leg_duration_.size(),
                           parameters_.target_swing_leg_orientation_.size());

  if (parameters_.side_ == +Side::LEFT)
  {
    eMatrixHom rf = bc_->getActualFootPose(+Side::LEFT);

    swing_leg_interpolator_.reset(new PoseReferenceMinJerkTopic(
        nh, bc_->getControllerDt(), "swing_leg_interpolator", "odom", "odom",
        rf.inverse() * bc_->getActualFootPose(+Side::RIGHT)));
  }
  else
  {
    eMatrixHom rf = bc_->getActualFootPose(+Side::RIGHT);

    swing_leg_interpolator_.reset(new PoseReferenceMinJerkTopic(
        nh, bc_->getControllerDt(), "swing_leg_interpolator", "odom", "odom",
        rf.inverse() * bc_->getActualFootPose(+Side::LEFT)));
  }

  ds_time_ = internal_time_ + ros::Duration(parameters_.ds_duration_);

  return true;
}

SStandOneLegAction::~SStandOneLegAction()
{
}

bool SStandOneLegAction::enterHook(const ros::Time &time)
{
  // Create interpolator for ICP from middle foot pose to target leg
  // Creat interpolator for ds force distribution weight

  bc_->setHybridControlFactor("leg_left_1_joint", 1.);
  bc_->setHybridControlFactor("leg_left_2_joint", 1.);
  bc_->setHybridControlFactor("leg_left_3_joint", 1.);
  bc_->setHybridControlFactor("leg_left_4_joint", 1.);
  bc_->setHybridControlFactor("leg_left_5_joint", 1.);
  bc_->setHybridControlFactor("leg_left_6_joint", 1.);
  bc_->setHybridControlFactor("leg_right_1_joint", 1.);
  bc_->setHybridControlFactor("leg_right_2_joint", 1.);
  bc_->setHybridControlFactor("leg_right_3_joint", 1.);
  bc_->setHybridControlFactor("leg_right_4_joint", 1.);
  bc_->setHybridControlFactor("leg_right_5_joint", 1.);
  bc_->setHybridControlFactor("leg_right_6_joint", 1.);

  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame = bc_->getActualFootPose(parameters_.side_);

  if (parameters_.side_ == +Side::RIGHT)
  {
    offset_ = eVector2(0.0, 0.02);
  }
  else
  {
    offset_ = eVector2(0.0, -0.02);
  }

  icp_interpolator_->initialize(
      { internal_time_, ds_time_ },
      { (local_coordinate_frame.inverse() * interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose)).translation().segment(0, 2), offset_ },
      { eVector2::Zero(), eVector2::Zero() },
      { eVector2::Zero(), eVector2::Zero() });

  if (parameters_.side_ == +Side::LEFT)
  {
    force_distribution_interpolator_->initialize({ internal_time_, ds_time_ },
                                                 { 0.5, 1. },
                                                 { 0., 0. },
                                                 { 0., 0. });
  }
  else
  {
    force_distribution_interpolator_->initialize({ internal_time_, ds_time_ },
                                                 { 0.5, 0. },
                                                 { 0., 0. },
                                                 { 0., 0. });
  }

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                           actual_left_foot_pose,
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                           actual_right_foot_pose,
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.));

  bc_->setActualSide(parameters_.side_);


  // Height Planning
  {
    if (parameters_.side_ == +Side::LEFT)
    {
      if (actual_left_foot_pose.translation().z() > actual_right_foot_pose.translation().z())
      {
        height_trajectory_.initialize(
            { ds_time_, ds_time_ + ros::Duration(parameters_.swing_leg_duration_[0]) },
            { actual_left_foot_pose.translation().z() - actual_right_foot_pose.translation().z(),0. },
            { 0., 0. },
            { 0., 0. });
      }
      else
      {
        height_trajectory_.initialize(
            { ds_time_, ds_time_ + ros::Duration(parameters_.swing_leg_duration_[0]) },
            { 0., 0. },
            { 0., 0. },
            { 0., 0. });
      }
    }
    else
    {
      if (actual_right_foot_pose.translation().z() > actual_left_foot_pose.translation().z())
      {
        height_trajectory_.initialize(
            { ds_time_, ds_time_ + ros::Duration(parameters_.swing_leg_duration_[0]) },
            { actual_right_foot_pose.translation().z() - actual_left_foot_pose.translation().z(),0. },
            { 0., 0. },
            { 0., 0. });
      }
      else
      {
        height_trajectory_.initialize(
            { ds_time_, ds_time_ + ros::Duration(parameters_.swing_leg_duration_[0]) },
            { 0., 0. },
            { 0., 0. },
            { 0., 0. });
      }
    }
  }

  return true;
}

bool SStandOneLegAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
  eMatrixHom local_coordinate_frame = bc_->getActualFootPose(parameters_.side_);

  if (internal_time_ < ds_time_)
  {
    eMatrixHom2d local_coordinate_frame_2d;
    pal::convert(local_coordinate_frame, local_coordinate_frame_2d);

    eVector2 local_target_dcm;
    eVector2 local_target_dcm_vel;
    eVector2 local_target_dcm_acc;
    icp_interpolator_->query(internal_time_,
                             local_target_dcm,
                             local_target_dcm_vel,
                             local_target_dcm_acc);

    double weight_distribution;
    double weight_distribution_d;
    double weight_distribution_dd;
    force_distribution_interpolator_->query(internal_time_,
                                            weight_distribution,
                                            weight_distribution_d,
                                            weight_distribution_dd);
    bc_->setWeightDistribution(weight_distribution);

    eVector2 global_target_dcm = local_coordinate_frame_2d * local_target_dcm;
    eVector2 global_target_cop = global_target_dcm;
    eVector2 global_target_dcm_vel = local_coordinate_frame_2d.rotation() * local_target_dcm_vel;

    double desired_height;
    double desired_heigh_vel;

    if (parameters_.side_ == +Side::RIGHT)
    {
      desired_heigh_vel = 0.;
      if (actual_left_foot_pose.translation().z() > actual_right_foot_pose.translation().z())
      {
        desired_height = actual_right_foot_pose.translation().z() + bc_->getParameters()->z_height_;
      }
      else
      {
        desired_height = actual_left_foot_pose.translation().z() + bc_->getParameters()->z_height_;
      }
    }
    else
    {
      desired_heigh_vel = 0.;
      if (actual_left_foot_pose.translation().z() < actual_right_foot_pose.translation().z())
      {
        desired_height = actual_left_foot_pose.translation().z() + bc_->getParameters()->z_height_;
      }
      else
      {
        desired_height = actual_right_foot_pose.translation().z() + bc_->getParameters()->z_height_;
      }
    }

    eVector3 targetCOM;
    eVector3 targetCOM_vel;
    targetCOM << global_target_dcm.x(), global_target_dcm.y(), desired_height;
    targetCOM_vel << global_target_dcm_vel.x(), global_target_dcm_vel.y(), desired_heigh_vel;

    control(bc_, rate_limiter_, targetCOM, targetCOM_vel, global_target_cop,
            parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
            targetCOP_unclamped_);

    double desiredYaw =
        extractYaw(interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose));

    bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
    bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  }
  else
  {
    bc_->setStanceLegIDs({ parameters_.side_ });
    if (parameters_.side_ == +Side::LEFT)
    {
      bc_->setSwingLegIDs({ Side::RIGHT });
    }
    else
    {
      bc_->setSwingLegIDs({ Side::LEFT });
    }

    swing_leg_interpolator_->integrate(time, bc_->getControllerDt());

    if (!swing_leg_configured_ ||
        (internal_time_ > ss_time_ &&
         swing_leg_pose_index_ < parameters_.swing_leg_duration_.size()))
    {
      eMatrixHom target_foot_pose =
          createMatrix(parameters_.target_swing_leg_orientation_[swing_leg_pose_index_],
                       parameters_.target_swing_leg_position_[swing_leg_pose_index_]);

      eMatrixHom target_relative_foot_pose = local_coordinate_frame.inverse() * target_foot_pose;

      ROS_INFO_STREAM("created swing leg trajectory, local target position: "
                      << target_relative_foot_pose.translation());
      swing_leg_interpolator_->setPoseTarget(
          target_relative_foot_pose,
          ros::Duration(parameters_.swing_leg_duration_[swing_leg_pose_index_]));

      // Turn on hybrid control on swing leg
      //      if (parameters_.side_ == +Side::LEFT)
      //      {
      //        bc_->setHybridControlFactor("leg_right_1_joint", 1.);
      //        bc_->setHybridControlFactor("leg_right_2_joint", 1.);
      //        bc_->setHybridControlFactor("leg_right_3_joint", 1.);
      //        bc_->setHybridControlFactor("leg_right_4_joint", 1.);
      //        bc_->setHybridControlFactor("leg_right_5_joint", 1.);
      //        bc_->setHybridControlFactor("leg_right_6_joint", 1.);
      //      }
      //      else
      //      {
      //        bc_->setHybridControlFactor("leg_left_1_joint", 1.);
      //        bc_->setHybridControlFactor("leg_left_2_joint", 1.);
      //        bc_->setHybridControlFactor("leg_left_3_joint", 1.);
      //        bc_->setHybridControlFactor("leg_left_4_joint", 1.);
      //        bc_->setHybridControlFactor("leg_left_5_joint", 1.);
      //        bc_->setHybridControlFactor("leg_left_6_joint", 1.);
      //      }

      swing_leg_configured_ = true;

      ss_time_ = internal_time_ +
                 ros::Duration(parameters_.swing_leg_duration_[swing_leg_pose_index_]);

      ++swing_leg_pose_index_;
    }

    eMatrixHom desired_local_swing_leg_pose = swing_leg_interpolator_->getDesiredPose();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> desired_local_swing_leg_velocity =
        swing_leg_interpolator_->getDesiredVelocity();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> desired_local_swing_leg_acceleration =
        swing_leg_interpolator_->getDesiredAcceleration();

    if (parameters_.side_ == +Side::LEFT)
    {
      bc_->setDesiredFootState(
          static_cast<int>(Side::RIGHT), local_coordinate_frame * desired_local_swing_leg_pose,
          local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.first,
          local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.first,
          local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.second,
          local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.second);
    }
    else
    {
      bc_->setDesiredFootState(
          static_cast<int>(Side::LEFT), local_coordinate_frame * desired_local_swing_leg_pose,
          local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.first,
          local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.first,
          local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.second,
          local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.second);
    }

    double desired_height;
    double desired_heigh_vel;

    if (swing_leg_pose_index_ == 1 && internal_time_ < ss_time_)
    {
      double height, height_d, height_dd;
      height_trajectory_.query(internal_time_, height, height_d, height_dd);
      desired_height = local_coordinate_frame.translation().z() +
                       bc_->getParameters()->z_height_ - height;
      desired_heigh_vel = height_d;
    }
    else
    {
      desired_heigh_vel = 0.;
      desired_height = local_coordinate_frame.translation().z() + bc_->getParameters()->z_height_;
    }

    eVector2 dcm_target;
    if (parameters_.side_ == +Side::RIGHT)
    {
      dcm_target = local_coordinate_frame.translation().segment(0, 2) + offset_;
    }
    else
    {
      dcm_target = local_coordinate_frame.translation().segment(0, 2) + offset_;
    }

    eVector3 targetCOM;
    targetCOM << dcm_target.x(), dcm_target.y(), desired_height;
    eVector3 targetCOM_vel;
    targetCOM_vel << 0.0, 0.0, desired_heigh_vel;

    control(bc_, rate_limiter_, targetCOM, targetCOM_vel,
            local_coordinate_frame.translation().segment(0, 2), parameters_.use_rate_limited_dcm_,
            targetCOP_rate_limited_unclamped_, targetCOP_unclamped_);

    double desiredYaw = extractYaw(local_coordinate_frame);

    bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
    bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  }

  internal_time_ += bc_->getControllerDt();

  return true;
}

bool SStandOneLegAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool SStandOneLegAction::endHook(const ros::Time &time)
{
  return true;
}
}
