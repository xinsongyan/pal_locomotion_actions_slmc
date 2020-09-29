/**
    @file
    @author  Alexander Sherikov

    @copyright (c) 2018 PAL Robotics SL. All Rights Reserved

    @brief
*/

#include <pal_locomotion_actions_slmc/static_walk.h>
#include <pal_locomotion/step_planner/step_planner_base.h>
#include <pal_locomotion/visualization/visualization.h>
#include <math_utils/geometry_tools.h>
#include <math_utils/eigen/rotation.h>
#include <pal_ros_utils/conversions.h>
#include <pal_utils/exception_utils.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <pal_ros_utils/visualization_tools.h>

namespace pal_locomotion
{
StaticWalkAction::StaticWalkAction(BController *bc)
  : active_walk_state_(eVector3(0, 0, 0), eVector3(0, 0, 0), ros::Duration(0.0),
                       ros::Duration(0.0), SupporType::DS)
{
  ROS_DEBUG_STREAM("Created static walk action");

  bc_ = bc;

  parameters_.readConfig<ariles::ros>(bc_->getNodeHandle(),
                                      "biped_parameters/static_walk_action_parameters");
}

StaticWalkAction::~StaticWalkAction()
{
}


bool StaticWalkAction::configure(ros::NodeHandle &nh, BController *bc,
                                 const property_bag::PropertyBag & /*parameters*/)
{
  ROS_DEBUG_STREAM("Configured balancing action");
  ROS_WARN("Property bag parameters are ignored in static walk action.");

  bc_ = bc;
  nh_ = nh;

  parameters_.readConfig<ariles::ros>(bc_->getNodeHandle(),
                                      "biped_parameters/static_walk_action_parameters");


  rate_limiter_.reset(new math_utils::HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  marray_index_ = 0;
  marray_.markers.reserve(1000);
  footstep_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>(
      nh, "footsteps", 1));

  foot_pose_trajectory_.reset(new pal_locomotion::SwingLegTrajectoryCatmulRom());

  REGISTER_VARIABLE("/introspection_data", "static_walk_state", &reg_stage_, &registered_variables_);

  return true;
}

// std::vector<std::pair<eMatrixHom, ros::Duration>> StaticWalkAction::createFootTraj(
//    const eMatrixHom &initial_foot_pose, const eMatrixHom &target_foot_pose)
//{
//  eVector3 foot_trajectory_midpoint =
//      (initial_foot_pose.translation() + target_foot_pose.translation()) / 2.0;
//  foot_trajectory_midpoint.z() =
//      initial_foot_pose.translation().z() + bc_->getParameters()->swing_leg_height_;

//  eQuaternion foot_trajectory_midrot =
//      eQuaternion(initial_foot_pose.rotation()).slerp(0.5,
//      eQuaternion(target_foot_pose.rotation()));

//  eMatrixHom foot_trajectory_midpose =
//      createMatrix(foot_trajectory_midrot, foot_trajectory_midpoint);

//  std::vector<std::pair<eMatrixHom, ros::Duration>> waypoints;
//  waypoints.push_back(std::make_pair(foot_trajectory_midpose,
//                                     ros::Duration(stage_duration_.toSec() / 2.0)));
//  return waypoints;
//}

eMatrixHom StaticWalkAction::getTargetPoseFromVel(const eMatrixHom &initial_stance_foot_pose)
{
  Eigen::Vector3d linear_velocity = active_walk_state_.getLinVel();
  Eigen::Vector3d angular_velocity = active_walk_state_.getAngVel();

  PAL_ASSERT_PERSIST(0.0 == linear_velocity.z(), "Vertical linear velocity cannot be set.");
  PAL_ASSERT_PERSIST(true == angular_velocity.head(2).isZero(),
                     "Only angular velocity about the vertical axis is supported.");
  if (angular_velocity.z() != 0)
  {
    PAL_ASSERT_PERSIST(
        0.0 == linear_velocity.y(),
        "If angular velocity is provided, only saggital linear velocity can be specified.");
  }


  // scale linear velocity of the feet if angular velocity is non zero
  //  if (angular_velocity.z() > 0.0)
  //  {
  //    if (swing_leg_ == +Side::LEFT)  // downscale inner velocity
  //    {
  //      linear_velocity.x() = linear_velocity.x() -
  //                            bc_->getParameters()->leg_kinematic_description_.hip_spacing_
  //                            /
  //                                2 * angular_velocity.z();
  //    }
  //    else  // upscale outer velocity
  //    {
  //      linear_velocity.x() = linear_velocity.x() +
  //                            bc_->getParameters()->leg_kinematic_description_.hip_spacing_
  //                            /
  //                                2 * angular_velocity.z();
  //    }
  //  }
  //  if (angular_velocity.z() < 0.0)
  //  {
  //    if (swing_leg_ == +Side::RIGHT)  // downscale inner velocity
  //    {
  //      linear_velocity.x() = linear_velocity.x() -
  //                            bc_->getParameters()->leg_kinematic_description_.hip_spacing_
  //                            /
  //                                2 * angular_velocity.z();
  //    }
  //    else  // upscale outer velocity
  //    {
  //      linear_velocity.x() = linear_velocity.x() +
  //                            bc_->getParameters()->leg_kinematic_description_.hip_spacing_
  //                            /
  //                                2 * angular_velocity.z();
  //    }
  //  }

  if ((fabs(angular_velocity.z()) > 0.0) && (linear_velocity.x() < 0.0))
    linear_velocity.x() = linear_velocity.x() / 2;

  eVector3 integrated_pos = stage_duration_.toSec() * linear_velocity;

  if ((angular_velocity.z() > 0.0) && (swing_leg_ == +pal_locomotion::Side::RIGHT))
  {
    integrated_pos.x() = 0.05;
  }
  if ((angular_velocity.z() < 0.0) && (swing_leg_ == +pal_locomotion::Side::LEFT))
  {
    integrated_pos.x() = 0.05;
  }

  if (swing_leg_ == +pal_locomotion::Side::RIGHT)
  {
    if (fabs(angular_velocity.z()) > 0.0)
      integrated_pos.y() -= parameters_.rotational_foot_separation_;
    else
      integrated_pos.y() -= parameters_.default_foot_separation_;
  }
  else
  {
    if (fabs(angular_velocity.z()) > 0.0)
      integrated_pos.y() += parameters_.rotational_foot_separation_;
    else
      integrated_pos.y() += parameters_.default_foot_separation_;
  }
  integrated_pos = initial_stance_foot_pose * integrated_pos;
  eMatrixHom target_foot_pose;
  target_foot_pose.translation() = integrated_pos;
  target_foot_pose.translation().z() = initial_stance_foot_pose.translation().z();
  target_foot_pose.linear() =
      initial_stance_foot_pose.linear() *
      Sophus::SO3d::exp(angular_velocity * stage_duration_.toSec()).unit_quaternion();
  return target_foot_pose;
}
bool StaticWalkAction::enterHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balancing action enter hook, time: " << time.toSec());

  bc_->setHybridControlFactor("leg_left_1_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_left_2_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_left_3_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_left_4_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_left_5_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_left_6_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_1_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_2_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_3_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_4_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_5_joint", parameters_.hybrid_factor_);
  bc_->setHybridControlFactor("leg_right_6_joint", parameters_.hybrid_factor_);

  std::vector<Side> stanceLegIds;
  stanceLegIds.push_back(+Side::LEFT);
  stanceLegIds.push_back(+Side::RIGHT);
  std::vector<Side> swingLegIds;
  bc_->setStanceLegIDs(stanceLegIds);
  bc_->setSwingLegIDs(swingLegIds);

  bc_->setWeightDistribution(0.5);

  bc_->setActualSupportType(+SupporType::DS);

  next_is_single_support_ = false;
  refpoint_ = REFPOINT_FOOT_MIDPOINT;
  stage_ = STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY;

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                           bc_->getActualFootPose(+Side::LEFT), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                           bc_->getActualFootPose(+Side::RIGHT), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  prev_side_ = +Side::LEFT;

  return true;
}

/// This function is called every cycle of the loop, until the end of the
/// Action()
bool StaticWalkAction::cycleHook(const ros::Time &time)
{
  reg_stage_ = static_cast<unsigned int>(stage_);
  Eigen::Isometry3d local_coordinate_frame;
  Eigen::Vector3d actual_com = bc_->getActualCOMPosition();
  Eigen::Isometry3d actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  Eigen::Isometry3d actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
  Eigen::Vector3d offset;

  switch (refpoint_)
  {
    case REFPOINT_FOOT_MIDPOINT:
    {
      local_coordinate_frame =
          interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
      offset << 0., 0., 0.;
    }
    break;

    case REFPOINT_FOOT_LEFT:
    {
      local_coordinate_frame = actual_left_foot_pose;
      offset << parameters_.static_walk_offset_[0], -parameters_.static_walk_offset_[1], 0.;
    }
    break;

    case REFPOINT_FOOT_RIGHT:
    {
      local_coordinate_frame = actual_right_foot_pose;
      offset << parameters_.static_walk_offset_[0], parameters_.static_walk_offset_[1], 0.;
    }
    break;

    default:
      PAL_THROW("Internal logic error: unexpected reference point type.");
  }
  local_coordinate_frame.linear() =
      pal::math_utils::matrixRollPitchYaw(0., 0., extractYaw(local_coordinate_frame));
  offset = eQuaternion(local_coordinate_frame.linear()) * offset;

  switch (stage_)
  {
    // send reference CoM point until new walk state is available.
    case STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT:
    {
      desired_com_state_.setZero();
      desired_com_state_.position_ = local_coordinate_frame.translation() + offset;
      desired_com_state_.position_.z() += bc_->getParameters()->z_height_;

      desired_base_angular_state_.setZero();
      desired_base_angular_state_.orientation_ =
          Eigen::Quaterniond(local_coordinate_frame.linear());
      desired_torso_angular_state_.setZero();
      desired_torso_angular_state_.orientation_ = desired_base_angular_state_.orientation_;

      VelocityCommandQuePtr walk_queue = bc_->getVelocityGenerator();
      // set next state
      if (walk_queue->commandQueSize() > 0)
      {
        active_walk_state_ = walk_queue->getCommand(0);
        walk_queue->popFirstCommand();

        switch (active_walk_state_.getSupport())
        {
          case SupporType::SS:
            next_is_single_support_ = true;
            if (active_walk_state_.hasSide())
            {
              if (active_walk_state_.getSide() == +Side::RIGHT)
              {
                refpoint_ = REFPOINT_FOOT_LEFT;
                support_leg_ = +Side::LEFT;
                swing_leg_ = +Side::RIGHT;
              }
              else
              {
                refpoint_ = REFPOINT_FOOT_RIGHT;
                support_leg_ = +Side::RIGHT;
                swing_leg_ = +Side::LEFT;
              }
            }
            else
            {
              if (prev_side_ == +Side::LEFT)
              {
                refpoint_ = REFPOINT_FOOT_LEFT;
                support_leg_ = +Side::LEFT;
                swing_leg_ = +Side::RIGHT;
              }
              else
              {
                refpoint_ = REFPOINT_FOOT_RIGHT;
                support_leg_ = +Side::RIGHT;
                swing_leg_ = +Side::LEFT;
              }
            }
            prev_side_ = swing_leg_;
            stage_ = STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY;
            break;

          case SupporType::DS:
            refpoint_ = REFPOINT_FOOT_MIDPOINT;
            break;

          default:
            PAL_THROW("Internal logic error: unexpected support type.");
        }
      }
    }
    break;

    // initialize CoM trajectory
    case STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY:
    {
      Eigen::Vector3d desiredCOM = local_coordinate_frame.translation() + offset;
      desiredCOM.z() += bc_->getParameters()->z_height_;

      double duration = (desiredCOM - actual_com).norm() / parameters_.preferred_com_velocity_;

      stage_timer_ = ros::Time(0.0);
      stage_duration_ = ros::Time(duration);

      com_trajectory_.initialize(
          std::make_pair(ros::Time(0), stage_duration_), std::make_pair(actual_com, desiredCOM),
          std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
          std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));

      initializeBaseTraj(bc_->getActualBaseOrientation(),
                         Eigen::Quaterniond(local_coordinate_frame.linear()));
      initializeTorsoTraj(bc_->getActualTorsoOrientation(),
                          Eigen::Quaterniond(local_coordinate_frame.linear()));

      double weight_distribution_target = 0.0;
      switch (refpoint_)
      {
        case REFPOINT_FOOT_LEFT:
          weight_distribution_target = 1.0;
          break;

        case REFPOINT_FOOT_RIGHT:
          weight_distribution_target = 0.0;
          break;

        case REFPOINT_FOOT_MIDPOINT:
          weight_distribution_target = 0.5;
          break;

        default:
          PAL_THROW("Internal logic error: unexpected reference point type while setting weight distribution.");
      }

      force_distribution_interpolator_.initialize(
          { stage_timer_, stage_duration_ },
          { bc_->getWeightDistribution(), weight_distribution_target }, { 0., 0. }, { 0., 0. });

      stage_ = STAGE_DOUBLE_SUPPORT_MOVE_COM;
    }
    // no break here

    // execute CoM trajectory
    case STAGE_DOUBLE_SUPPORT_MOVE_COM:
    {
      com_trajectory_.query(stage_timer_, desired_com_state_.position_,
                            desired_com_state_.linear_velocity_,
                            desired_com_state_.linear_acceleration_);
      //      desired_com_state_.linear_velocity_ =
      //          local_coordinate_frame_.rotation() *
      //          desired_com_state_.linear_velocity_;
      //      desired_com_state_.linear_acceleration_ =
      //          local_coordinate_frame_.rotation() *
      //          desired_com_state_.linear_acceleration_;

      base_orientation_trajectory_.query(stage_timer_, desired_base_angular_state_.orientation_,
                                         desired_base_angular_state_.angular_velocity_,
                                         desired_base_angular_state_.angular_acceleration_);

      torso_orientation_trajectory_.query(stage_timer_, desired_torso_angular_state_.orientation_,
                                          desired_torso_angular_state_.angular_velocity_,
                                          desired_torso_angular_state_.angular_acceleration_);

      double weight_distribution;
      double weight_distribution_d;
      double weight_distribution_dd;

      force_distribution_interpolator_.query(stage_timer_, weight_distribution,
                                             weight_distribution_d, weight_distribution_dd);
      bc_->setWeightDistribution(weight_distribution);

      stage_timer_ += bc_->getControllerDt();

      if (stage_timer_ > stage_duration_)
      {
        stage_ = STAGE_COM_HOLD_REFPOINT;
      }
    }
    break;

    // send reference CoM point until error is below the given threshold
    case STAGE_COM_HOLD_REFPOINT:
    {
      desired_com_state_.setZero();
      desired_com_state_.position_ = local_coordinate_frame.translation() + offset;
      desired_com_state_.position_.z() += bc_->getParameters()->z_height_;

      desired_base_angular_state_.setZero();
      desired_base_angular_state_.orientation_ =
          Eigen::Quaterniond(local_coordinate_frame.linear());
      desired_torso_angular_state_.setZero();
      desired_torso_angular_state_.orientation_ = desired_base_angular_state_.orientation_;

      // Compute the com_error only in 2D
      Eigen::Vector3d com_diff = desired_com_state_.position_ - actual_com;
      com_diff.z() = 0.0;
      desired_com_state_.position_ += parameters_.integral_correction_ * com_diff;

      bool accept_error;
      if(refpoint_ == REFPOINT_FOOT_MIDPOINT)
          accept_error  = (com_diff.norm() <= parameters_.acceptable_com_ds_reference_error_) ? true : false;
      else
          accept_error  = (com_diff.norm() <= parameters_.acceptable_com_ss_reference_error_) ? true : false;

      if (accept_error)
      {
        if (true == next_is_single_support_)
        {
          stage_ = STAGE_SINGLE_SUPPORT_SWING_INITIALIZE;
        }
        else
        {
          stage_ = STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT;
        }
      }
      planned_footstep_ = true;
    }
    break;

    // initialize swing motion
    case STAGE_SINGLE_SUPPORT_SWING_INITIALIZE:
    {
      stage_timer_ = ros::Time(0.0);

      Eigen::Isometry3d swing_leg_foot_pose = bc_->getActualFootPose(swing_leg_);
      Eigen::Isometry3d stance_leg_foot_pose = bc_->getActualFootPose(support_leg_);

      waypoints_.clear();

      if (planned_footstep_)
      {
        stage_duration_ = ros::Time(active_walk_state_.getStanceDuration().toSec());

        if (true == active_walk_state_.hasPose())
          target_foot_pose_ = active_walk_state_.getPose();
        else
          target_foot_pose_ = getTargetPoseFromVel(stance_leg_foot_pose);

        waypoints_ = active_walk_state_.getWaypoints();

        //        if (waypoints_.empty())
        //          waypoints_ = createFootTraj(swing_leg_foot_pose, target_foot_pose_);

        over_timer_ = ros::Time(0);
      }

      eVector3 desiredCOM = local_coordinate_frame.translation() + offset;
      desiredCOM.z() += bc_->getParameters()->z_height_;
      eVector3 actual_fixedCOM = desiredCOM;
      if (target_foot_pose_.translation().z() < local_coordinate_frame.translation().z())
      {
        desiredCOM.z() +=
            (target_foot_pose_.translation().z() - local_coordinate_frame.translation().z());
      }
      if(planned_footstep_)
      {
          prev_COM_z_ = desiredCOM.z();
      }
      else
      {
          desiredCOM.z() = prev_COM_z_;
      }

      com_trajectory_.initialize(
          std::make_pair(ros::Time(0), ros::Time(active_walk_state_.getStanceDuration().toSec())),
          std::make_pair(actual_fixedCOM, desiredCOM),
          std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
          std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));

      waypoints_.push_back(CartesianTrajectoryPoint(
          target_foot_pose_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          ros::Duration(active_walk_state_.getStanceDuration().toSec())));
      clampWaypoints(stance_leg_foot_pose, time);

      //      actual_wypt_ = 0;
      //      foot_pose_trajectory_.configure(
      //          nh_, bc_->getControllerDt(), "swing_leg_up_interpolator", "odom",
      //          "odom",
      //          bc_->getActualFootPose(swing_leg_), Eigen::Vector3d::Zero(),
      //          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      //          Eigen::Vector3d::Zero(),
      //          waypoints_[actual_wypt_].first, Eigen::Vector3d::Zero(),
      //          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      //          Eigen::Vector3d::Zero());
      //      foot_pose_trajectory_.setPoseTarget(waypoints_[actual_wypt_].first,
      //                                          waypoints_[actual_wypt_].second);


      if (planned_footstep_)
      {
        double swing_leg_height = bc_->getParameters()->swing_leg_height_;

        foot_pose_trajectory_->create(stage_timer_, swing_leg_height, swing_leg_foot_pose,
                                      waypoints_);
      }
      else
      {
        eQuaternion q1(prev_target_foot_pose_.rotation());
        eQuaternion q2(target_foot_pose_.rotation());
        double swing_leg_height = 0.0;

        foot_pose_trajectory_->create(stage_timer_, swing_leg_height,
                                      prev_target_foot_pose_, waypoints_);
      }

      for (size_t i = 0; i < waypoints_.size() - 1; i++)
      {
        PAL_ASSERT_PERSIST_LESS(waypoints_[i].getTime().toSec(),
                                waypoints_[i + 1].getTime().toSec());
      }

      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        eVector3 color;
        if (swing_leg_ == +pal_locomotion::Side::LEFT)
          color = RED;
        else
          color = GREEN;

        pal_robot_tools::publish_box(
            waypoints_[i].getPose(), color,
            eVector3(bc_->getParameters()->foot_description_.foot_length_,
                     bc_->getParameters()->foot_description_.foot_with_,
                     bc_->getParameters()->foot_description_.foot_height_),
            "odom", "footsteps", time, marray_, marray_index_);
      }

      pal_robot_tools::publish_box(
          stance_leg_foot_pose, BLUE,
          eVector3(bc_->getParameters()->foot_description_.foot_length_,
                   bc_->getParameters()->foot_description_.foot_with_,
                   bc_->getParameters()->foot_description_.foot_height_),
          "odom", "footsteps", time, marray_, marray_index_);

      pal_robot_tools::publish_box(
          swing_leg_foot_pose, ORANGE,
          eVector3(bc_->getParameters()->foot_description_.foot_length_,
                   bc_->getParameters()->foot_description_.foot_with_,
                   bc_->getParameters()->foot_description_.foot_height_),
          "odom", "footsteps", time, marray_, marray_index_);

      if (footstep_pub_->trylock())
      {
        footstep_pub_->msg_ = marray_;
        footstep_pub_->unlockAndPublish();
      }
      marray_.markers.clear();
      marray_index_ = 0;

      // interpolate between the current support and landing target
      // pose, then extract yaw (to prevent swaying of the torso)
      Eigen::Isometry3d target_orientation =
          interpolateBetweenTransforms(local_coordinate_frame, target_foot_pose_);
      target_orientation.linear() =
          pal::math_utils::matrixRollPitchYaw(0., 0., extractYaw(target_orientation));

      initializeTorsoTraj(bc_->getActualBaseOrientation(),
                          Eigen::Quaterniond(target_orientation.linear()));
      initializeBaseTraj(bc_->getActualBaseOrientation(),
                         Eigen::Quaterniond(target_orientation.linear()));

      const std::vector<Side> stanceLegIds = { support_leg_ };
      const std::vector<Side> swingLegIds = { swing_leg_ };
      bc_->setStanceLegIDs(stanceLegIds);
      bc_->setSwingLegIDs(swingLegIds);

      bc_->setActualSupportType(+SupporType::SS);

      stage_ = STAGE_SINGLE_SUPPORT_SWING;
    }
    // no break here

    // execute swing motion
    case STAGE_SINGLE_SUPPORT_SWING:
    {
      com_trajectory_.query(stage_timer_, desired_com_state_.position_,
                            desired_com_state_.linear_velocity_,
                            desired_com_state_.linear_acceleration_);

      base_orientation_trajectory_.query(stage_timer_, desired_base_angular_state_.orientation_,
                                         desired_base_angular_state_.angular_velocity_,
                                         desired_base_angular_state_.angular_acceleration_);


      torso_orientation_trajectory_.query(stage_timer_, desired_torso_angular_state_.orientation_,
                                          desired_torso_angular_state_.angular_velocity_,
                                          desired_torso_angular_state_.angular_acceleration_);

      //      ros::Time final_time = foot_pose_trajectory_.getFinalTime();

      //      if (stage_timer_ > final_time)
      //      {
      //        if (actual_wypt_ < waypoints_.size())
      //          actual_wypt_++;

      //        foot_pose_trajectory_.setPoseTarget(waypoints_[actual_wypt_].first,
      //                                            waypoints_[actual_wypt_].second);
      //      }

      //      foot_pose_trajectory_.integrate(stage_timer_, bc_->getControllerDt());
      //      Eigen::Isometry3d desired_foot_pose =
      //      foot_pose_trajectory_.getDesiredPose();
      //      std::pair<eVector3, eVector3> foot_velocities =
      //          foot_pose_trajectory_.getDesiredVelocity();
      //      std::pair<eVector3, eVector3> foot_accelerations =
      //          foot_pose_trajectory_.getDesiredAcceleration();

      Eigen::Isometry3d desired_foot_pose;
      std::pair<eVector3, eVector3> foot_velocities;
      std::pair<eVector3, eVector3> foot_accelerations;
      foot_pose_trajectory_->getDesiredState(stage_timer_, desired_foot_pose, foot_velocities, foot_accelerations);
          foot_pose_trajectory_->getDesiredAcceleration(stage_timer_);

      bc_->setDesiredFootState(static_cast<int>(swing_leg_), desired_foot_pose,
                               foot_velocities.first, foot_accelerations.first,
                               foot_velocities.second, foot_accelerations.second);

      if ((!planned_footstep_) || (stage_timer_.toSec() > active_walk_state_.getStanceDuration().toSec() / 2.0))
      {
        if (bc_->isFTVerticalContact(swing_leg_, bc_->getParameters()->minimum_contact_force_))
        {
          std::vector<Side> stanceLegIds;
          stanceLegIds.push_back(+Side::LEFT);
          stanceLegIds.push_back(+Side::RIGHT);
          std::vector<Side> swingLegIds;
          bc_->setStanceLegIDs(stanceLegIds);
          bc_->setSwingLegIDs(swingLegIds);

          next_is_single_support_ = false;
          refpoint_ = REFPOINT_FOOT_MIDPOINT;
          stage_ = STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY;
          stage_timer_ = ros::Time(0.0);
        }
      }

      stage_timer_ += bc_->getControllerDt();

      if (stage_timer_ > ros::Time(active_walk_state_.getStanceDuration().toSec()))
      {
        over_timer_ += bc_->getControllerDt();
        stage_timer_ = ros::Time(active_walk_state_.getStanceDuration().toSec());

        if ((over_timer_ >= ros::Time(0.2 * active_walk_state_.getStanceDuration().toSec())) &&
            (!bc_->isFTVerticalContact(swing_leg_, bc_->getParameters()->minimum_contact_force_)))
        {
          stage_ = STAGE_SINGLE_SUPPORT_SWING_INITIALIZE;
          prev_target_foot_pose_ = target_foot_pose_;
          target_foot_pose_.translation() =
              target_foot_pose_ *
              eVector3(0, 0, -bc_->getParameters()->postmature_contact_height_);
          WalkingStepCommand new_command(target_foot_pose_, ros::Duration(0.0),
                                         ros::Duration(stage_duration_.toSec() / 2.),
                                         swing_leg_);
          new_command.setStanceTime(ros::Duration(stage_duration_.toSec() / 2.));
          active_walk_state_ = new_command;
          planned_footstep_ = false;
        }
      }
    }
    break;

    default:
      PAL_THROW("Internal logic error: unexpected tage type.");
  }

  bc_->setDesiredBaseOrientation(desired_base_angular_state_.orientation_);
  bc_->setDesiredBaseAngularVelocity(desired_base_angular_state_.angular_velocity_);
  bc_->setDesiredBaseAngularAcceleration(desired_base_angular_state_.angular_acceleration_);

  bc_->setDesiredTorsoOrientation(desired_torso_angular_state_.orientation_);
  bc_->setDesiredTorsoAngularVelocity(desired_torso_angular_state_.angular_velocity_);
  bc_->setDesiredTorsoAngularAcceleration(desired_torso_angular_state_.angular_acceleration_);

  //  ROS_WARN_STREAM("Desired com " << desired_com_state_.position_.transpose());
  control(bc_, rate_limiter_, desired_com_state_.position_,
          desired_com_state_.linear_velocity_, desired_com_state_.position_.segment(0, 2),
          parameters_.use_rate_limited_dcm_, actual_com, bc_->getActualCOMVelocity2d(),
          targetCOP_rate_limited_unclamped_, targetCOP_unclamped_);

  return true;
}

bool StaticWalkAction::isOverHook(const ros::Time &time)
{
  // If we're in a safe stage, and there's a task in the queue, we can end.
  if ((stage_ == STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT) &&
      (bc_->getStateMachine()->queue_size() > 1))
  {
    return true;
  }
  return false;
}

/// when isOver()=true, this function is called and the action is removed from
/// the queue.
bool StaticWalkAction::endHook(const ros::Time &time)
{
  ROS_INFO_STREAM("Balancing action end hook, time: " << time.toSec());
  return true;
}

void StaticWalkAction::initializeTorsoTraj(const eQuaternion &from, const eQuaternion &to)
{
  torso_orientation_trajectory_.initialize(
      std::make_pair(ros::Time(0), stage_duration_), std::make_pair(from, to),
      std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}

void StaticWalkAction::initializeBaseTraj(const eQuaternion &from, const eQuaternion &to)
{
  base_orientation_trajectory_.initialize(
      std::make_pair(ros::Time(0), stage_duration_), std::make_pair(from, to),
      std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}

void StaticWalkAction::clampWaypoints(const eMatrixHom &support_leg_pose, const ros::Time &time)
{
  eVector3 min_clamp;
  eVector3 max_clamp;
  if (support_leg_ == +Side::RIGHT)
  {
    min_clamp << parameters_.step_saggital_bounds_(0),
        parameters_.step_coronal_bounds_(0), parameters_.step_axial_bounds_(0);
    max_clamp << parameters_.step_saggital_bounds_(1),
        parameters_.step_coronal_bounds_(1), parameters_.step_axial_bounds_(1);
  }
  else
  {
    min_clamp << parameters_.step_saggital_bounds_(0),
        -parameters_.step_coronal_bounds_(1), parameters_.step_axial_bounds_(0);
    max_clamp << parameters_.step_saggital_bounds_(1),
        -parameters_.step_coronal_bounds_(0), parameters_.step_axial_bounds_(1);
  }

  pal_robot_tools::publish_line(support_leg_pose * eVector3(min_clamp.x(), min_clamp.y(), 0.0),
                                support_leg_pose * eVector3(max_clamp.x(), min_clamp.y(), 0.0),
                                YELLOW, "odom", "footsteps", time, marray_, marray_index_);
  pal_robot_tools::publish_line(support_leg_pose * eVector3(min_clamp.x(), max_clamp.y(), 0.0),
                                support_leg_pose * eVector3(max_clamp.x(), max_clamp.y(), 0.0),
                                YELLOW, "odom", "footsteps", time, marray_, marray_index_);
  pal_robot_tools::publish_line(support_leg_pose * eVector3(min_clamp.x(), min_clamp.y(), 0.0),
                                support_leg_pose * eVector3(min_clamp.x(), max_clamp.y(), 0.0),
                                YELLOW, "odom", "footsteps", time, marray_, marray_index_);
  pal_robot_tools::publish_line(support_leg_pose * eVector3(max_clamp.x(), min_clamp.y(), 0.0),
                                support_leg_pose * eVector3(max_clamp.x(), max_clamp.y(), 0.0),
                                YELLOW, "odom", "footsteps", time, marray_, marray_index_);

  for (size_t i = 0; i < waypoints_.size(); i++)
  {
    eMatrixHom local_wypt = support_leg_pose.inverse() * waypoints_[i].getPose();
    local_wypt.translation() = CLAMP(local_wypt.translation(), min_clamp, max_clamp);
    double initial_yaw = extractYaw(local_wypt);
    double yaw = 0.0;
    if (support_leg_ == +Side::RIGHT)
    {
      yaw = CLAMP(initial_yaw, parameters_.step_rotational_bounds_(0),
                  parameters_.step_rotational_bounds_(1));
    }
    else
    {
      yaw = CLAMP(initial_yaw, -parameters_.step_rotational_bounds_(1),
                  -parameters_.step_rotational_bounds_(0));
    }
    eMatrixHom m = HommatrixRollPitchYaw(0.0, 0.0, yaw - initial_yaw);
    m.translation() << 0, 0, 0;
    local_wypt = m * local_wypt;
    waypoints_[i].setPose(support_leg_pose * local_wypt);
  }
}
}
