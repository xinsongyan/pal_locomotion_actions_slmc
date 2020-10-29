#include <pal_locomotion_actions_slmc/stand_down_leg_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace pal_robot_tools;
using namespace math_utils;

namespace pal_locomotion
{
StandDownLegAction::StandDownLegAction()
  : internal_time_(ros::Time(0))
  , swing_leg_configured_(false)
  , icp_interpolator_configured_(false)
{
}

bool StandDownLegAction::configure(ros::NodeHandle &nh, BController *bc,
                                   const property_bag::PropertyBag &pb)
{
  bc_ = bc;

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  icp_interpolator_.reset(new MinJerkGenerator2D());

  force_distribution_interpolator_.reset(new MinJerkGenerator());

  pb.getPropertyValue<double>("ds_duration", parameters_.ds_duration_,
                              property_bag::RetrievalHandling::THROW);

  pb.getPropertyValue<double>("swing_leg_duration", parameters_.swing_leg_duration_,
                              property_bag::RetrievalHandling::THROW);
  pb.getPropertyValue<eQuaternion>("target_swing_leg_orientation",
                                   parameters_.target_swing_leg_orientation_,
                                   property_bag::RetrievalHandling::THROW);

  pb.getPropertyValue<eVector3>("target_swing_leg_position", parameters_.target_swing_leg_position_,
                                property_bag::RetrievalHandling::THROW);

  std::string side_str;
  pb.getPropertyValue<std::string>("side", side_str, property_bag::RetrievalHandling::THROW);
  parameters_.side_ = Side::_from_string(side_str.c_str());

  ds_time_ = internal_time_ + ros::Duration(parameters_.swing_leg_duration_);
  ss_time_ = internal_time_ + ros::Duration(parameters_.swing_leg_duration_) +
             ros::Duration(parameters_.ds_duration_);

  if (parameters_.side_ == +Side::LEFT)
  {
    eMatrixHom rf = bc_->getActualFootPose(+Side::RIGHT);

    swing_leg_interpolator_.reset(new PoseReferenceMinJerkTopic(
        nh, bc_->getControllerDt(), "swing_leg_interpolator", "odom", "odom",
        rf.inverse() * bc_->getActualFootPose(+Side::LEFT)));

    force_distribution_interpolator_->initialize({ ds_time_, ss_time_ }, { 0.0, 0.5 },
                                                 { 0., 0. }, { 0., 0. });
  }
  else
  {
    eMatrixHom rf = bc_->getActualFootPose(+Side::LEFT);

    swing_leg_interpolator_.reset(new PoseReferenceMinJerkTopic(
        nh, bc_->getControllerDt(), "swing_leg_interpolator", "odom", "odom",
        rf.inverse() * bc_->getActualFootPose(+Side::RIGHT)));

    force_distribution_interpolator_->initialize({ ds_time_, ss_time_ }, { 1.0, 0.5 },
                                                 { 0., 0. }, { 0., 0. });
  }

  return true;
}

StandDownLegAction::~StandDownLegAction()
{
}

bool StandDownLegAction::enterHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

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

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT), actual_left_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT), actual_right_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  // Height Planning
  {
    if (parameters_.side_ == +Side::LEFT)
    {
      if (actual_right_foot_pose.translation().z() >
          parameters_.target_swing_leg_position_.z())
      {
        height_trajectory_.initialize({ internal_time_, ds_time_ },
                                      { 0.,
                                        parameters_.target_swing_leg_position_.z() -
                                            actual_right_foot_pose.translation().z() },
                                      { 0., 0. }, { 0., 0. });
      }
      else
      {
        height_trajectory_.initialize({ internal_time_, ds_time_ }, { 0., 0. },
                                      { 0., 0. }, { 0., 0. });
      }
    }
    else
    {
      if (actual_left_foot_pose.translation().z() > parameters_.target_swing_leg_position_.z())
      {
        height_trajectory_.initialize({ internal_time_, ds_time_ },
                                      { 0.,
                                        parameters_.target_swing_leg_position_.z() -
                                            actual_left_foot_pose.translation().z() },
                                      { 0., 0. }, { 0., 0. });
      }
      else
      {
        height_trajectory_.initialize({ internal_time_, ds_time_ }, { 0., 0. },
                                      { 0., 0. }, { 0., 0. });
      }
    }
  }

  if (parameters_.side_ == +Side::RIGHT)
  {
    offset_ = eVector2(0.0, -0.02);
  }
  else
  {
    offset_ = eVector2(0.0, 0.02);
  }

  height_ = 0.0;

  return true;
}

bool StandDownLegAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
  eMatrixHom local_coordinate_frame;
  if (parameters_.side_ == +Side::LEFT)
  {
    local_coordinate_frame = actual_right_foot_pose;
  }
  else
  {
    local_coordinate_frame = actual_left_foot_pose;
  }

  if (internal_time_ < ds_time_)
  {
    swing_leg_interpolator_->integrate(time, bc_->getControllerDt());

    if (!swing_leg_configured_)
    {
      eMatrixHom target_foot_pose = createMatrix(parameters_.target_swing_leg_orientation_,
                                                 parameters_.target_swing_leg_position_);

      eMatrixHom target_relative_foot_pose = local_coordinate_frame.inverse() * target_foot_pose;

      ROS_INFO_STREAM("created swing leg trajectory, local target position: "
                      << target_relative_foot_pose.translation());
      swing_leg_interpolator_->setPoseTarget(target_relative_foot_pose,
                                             ros::Duration(parameters_.swing_leg_duration_));

      swing_leg_configured_ = true;
    }

    eMatrixHom desired_local_swing_leg_pose = swing_leg_interpolator_->getDesiredPose();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> desired_local_swing_leg_velocity =
        swing_leg_interpolator_->getDesiredVelocity();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> desired_local_swing_leg_acceleration =
        swing_leg_interpolator_->getDesiredAcceleration();

    bc_->setDesiredFootState(
        static_cast<int>(parameters_.side_), local_coordinate_frame * desired_local_swing_leg_pose,
        local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.first,
        local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.first,
        local_coordinate_frame.rotation() * desired_local_swing_leg_velocity.second,
        local_coordinate_frame.rotation() * desired_local_swing_leg_acceleration.second);

    double height_d = 0, height_dd = 0;
    height_trajectory_.query(internal_time_, height_, height_d, height_dd);

    if (parameters_.side_ == +Side::RIGHT)
    {
      desired_height_ = actual_left_foot_pose.translation().z() +
                        bc_->getParameters()->z_height_ + height_;
    }
    else
    {
      desired_height_ = actual_right_foot_pose.translation().z() +
                        bc_->getParameters()->z_height_ + height_;
    }

    eVector2 dcm_target = local_coordinate_frame.translation().segment(0, 2) + offset_;

    if (!bc_->isFTVerticalContact(parameters_.side_, bc_->getParameters()->minimum_contact_force_))
    {
        eVector3 targetCOM;
        targetCOM << dcm_target.x(), dcm_target.y(), desired_height_;
        eVector3 targetCOM_vel;
        targetCOM_vel << 0.0, 0.0, height_d;

        control(bc_, rate_limiter_, targetCOM, targetCOM_vel, local_coordinate_frame.translation().segment(0, 2),
                parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
                targetCOP_unclamped_);
    }
    else
    {
      internal_time_ = ds_time_;
    }

    if (((ds_time_ - internal_time_) <= bc_->getControllerDt()) &&
        (!bc_->isFTVerticalContact(parameters_.side_, bc_->getParameters()->minimum_contact_force_)))
    {
      double time_diff = 0.5 * ds_time_.toSec();
      internal_time_ = ros::Time(time_diff);
      parameters_.swing_leg_duration_ = time_diff;
      parameters_.target_swing_leg_position_.z() -= bc_->getParameters()->postmature_contact_height_;
      eMatrixHom target_foot_pose = createMatrix(parameters_.target_swing_leg_orientation_,
                                                 parameters_.target_swing_leg_position_);

      eMatrixHom target_relative_foot_pose = local_coordinate_frame.inverse() * target_foot_pose;

      swing_leg_interpolator_->setPoseTarget(
          target_relative_foot_pose, ros::Duration(0.5 * parameters_.swing_leg_duration_));

      if (parameters_.side_ == +Side::LEFT)
      {
        if (actual_right_foot_pose.translation().z() >
            parameters_.target_swing_leg_position_.z())
        {
          height_trajectory_.initialize({ internal_time_, ds_time_ },
                                        { 0.,
                                          parameters_.target_swing_leg_position_.z() -
                                              actual_right_foot_pose.translation().z() },
                                        { 0., 0. }, { 0., 0. });
        }
        else
        {
          height_trajectory_.initialize({ internal_time_, ds_time_ }, { 0., 0. },
                                        { 0., 0. }, { 0., 0. });
        }
      }
      else
      {
        if (actual_left_foot_pose.translation().z() >
            parameters_.target_swing_leg_position_.z())
        {
          height_trajectory_.initialize({ internal_time_, ds_time_ },
                                        { 0.,
                                          parameters_.target_swing_leg_position_.z() -
                                              actual_left_foot_pose.translation().z() },
                                        { 0., 0. }, { 0., 0. });
        }
        else
        {
          height_trajectory_.initialize({ internal_time_, ds_time_ }, { 0., 0. },
                                        { 0., 0. }, { 0., 0. });
        }
      }
    }

    double desiredYaw = extractYaw(local_coordinate_frame);

    bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
    bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  }
  else if (internal_time_ >= ds_time_ && internal_time_ < ss_time_)
  {
    // Turn off hybrid control on swing leg
    //    if (parameters_.side_ == +Side::LEFT)
    //    {
    //      bc_->setHybridControlFactor("leg_left_1_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_2_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_3_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_4_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_5_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_6_joint", 0.);
    //    }
    //    else
    //    {
    //      bc_->setHybridControlFactor("leg_right_1_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_2_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_3_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_4_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_5_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_6_joint", 0.);
    //    }
    bc_->setStanceLegIDs({ Side::LEFT, Side::RIGHT });
    bc_->setSwingLegIDs({});
    eMatrixHom local_coordinate_frame_aux =
        interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);

    eMatrixHom2d local_coordinate_frame_2d;
    pal::convert(local_coordinate_frame_aux, local_coordinate_frame_2d);

    if (!icp_interpolator_configured_)
    {
      if (parameters_.side_ == +Side::LEFT)
      {
        icp_interpolator_->initialize(
            { internal_time_, ss_time_ },
            { (local_coordinate_frame_aux.inverse() * actual_right_foot_pose)
                      .translation()
                      .segment(0, 2) +
                  offset_,
              eVector2::Zero() },
            { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(), eVector2::Zero() });
      }
      else
      {
        icp_interpolator_->initialize(
            { internal_time_, ss_time_ },
            { (local_coordinate_frame_aux.inverse() * actual_left_foot_pose).translation().segment(0, 2) +
                  offset_,
              eVector2::Zero() },
            { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(), eVector2::Zero() });
      }
      //      icp_interpolator_->initialize(
      //          { internal_time_, ss_time_ },
      //          { bc_->getActualDCM().segment(0, 2),
      //          local_coordinate_frame_2d.translation() },
      //          { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(),
      //          eVector2::Zero() });
      icp_interpolator_configured_ = true;
    }
    eVector2 local_target_dcm;
    eVector2 local_target_dcm_vel;
    eVector2 local_target_dcm_acc;
    icp_interpolator_->query(internal_time_, local_target_dcm, local_target_dcm_vel,
                             local_target_dcm_acc);

    eVector2 global_target_dcm = local_coordinate_frame_2d * local_target_dcm;
    eVector2 global_target_cop = global_target_dcm;
    eVector2 global_target_dcm_vel = local_coordinate_frame_2d.rotation() * local_target_dcm_vel;

    //    eVector2 global_target_dcm;
    //    eVector2 global_target_cop = global_target_dcm;
    //    eVector2 global_target_dcm_vel;
    //    eVector2 global_target_dcm_acc;
    //    icp_interpolator_->query(internal_time_, global_target_dcm,
    //    global_target_dcm_vel,
    //                             global_target_dcm_acc);

    double weight_distribution;
    double weight_distribution_d;
    double weight_distribution_dd;
    force_distribution_interpolator_->query(internal_time_, weight_distribution,
                                            weight_distribution_d, weight_distribution_dd);
    bc_->setWeightDistribution(weight_distribution);


    double desired_height_d = 0;

    eVector3 targetCOM;
    targetCOM << global_target_dcm.x(), global_target_dcm.y(), desired_height_;
    eVector3 targetCOM_vel;
    targetCOM_vel << global_target_dcm_vel.x(), global_target_dcm_vel.y(), desired_height_d;

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
    // Turn off hybrid control on swing leg
    //    if (parameters_.side_ == +Side::LEFT)
    //    {
    //      bc_->setHybridControlFactor("leg_left_1_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_2_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_3_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_4_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_5_joint", 0.);
    //      bc_->setHybridControlFactor("leg_left_6_joint", 0.);
    //    }
    //    else
    //    {
    //      bc_->setHybridControlFactor("leg_right_1_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_2_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_3_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_4_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_5_joint", 0.);
    //      bc_->setHybridControlFactor("leg_right_6_joint", 0.);
    //    }

    eMatrixHom actual_left_foot_pose_aux = bc_->getActualFootPose(+Side::LEFT);
    eMatrixHom actual_right_foot_pose_aux = bc_->getActualFootPose(+Side::RIGHT);

    eMatrixHom local_coordinate_frame_aux =
        interpolateBetweenTransforms(actual_left_foot_pose_aux, actual_right_foot_pose_aux);

    double desired_yaw = extractYaw(local_coordinate_frame_aux);

    eMatrixHom2d local_coordinate_frame_2d;
    pal::convert(local_coordinate_frame_aux, local_coordinate_frame_2d);

    Eigen::Vector2d referenceCOP = local_coordinate_frame_2d.translation();
    Eigen::Vector2d targetDCM = local_coordinate_frame_2d.translation();
    Eigen::Vector2d targetDCM_vel = Eigen::Vector2d::Zero();

    double desired_height_d  = 0.;

    eVector3 targetCOM;
    targetCOM << targetDCM.x(), targetDCM.y(), desired_height_;
    eVector3 targetCOM_vel;
    targetCOM_vel << targetDCM_vel.x(), targetDCM_vel.y(), desired_height_d;

    control(bc_, rate_limiter_, targetCOM, targetCOM_vel, referenceCOP,
            parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
            targetCOP_unclamped_);

    bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desired_yaw)));
    bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desired_yaw)));
  }

  internal_time_ += bc_->getControllerDt();

  return true;
}

bool StandDownLegAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool StandDownLegAction::endHook(const ros::Time &time)
{
  return true;
}
}
