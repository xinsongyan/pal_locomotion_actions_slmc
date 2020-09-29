/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#pragma once

#include <pal_locomotion/biped_controller.h>
#include <pal_locomotion/state_machine/walking_action_base.h>
#include <ariles/ariles_all.h>
#include <math_utils/physical_states/rigidbody.h>
#include <pal_ros_utils/reference/pose/pose_reference_abstract.h>
#include <pal_ros_utils/pluginlib_helpers.h>
#include <pal_physics_utils/rbcomposite/trajectory.h>

#include <math_utils/physical_states/pointmass.h>
#include <math_utils/physical_states/rigidbody.h>
#include <math_utils/trajectory/min_jerk_generator.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>

#include <pal_locomotion/leg_trajectory/swing_leg_trajectory_catmulrom.h>
#include <pal_statistics/pal_statistics_macros.h>

namespace pal_locomotion
{
class StaticWalkActionParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "StaticWalkActionParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(preferred_com_velocity, double)                                    \
  ARILES_TYPED_ENTRY_(acceptable_com_ds_reference_error, double)                         \
  ARILES_TYPED_ENTRY_(acceptable_com_ss_reference_error, double)                         \
  ARILES_TYPED_ENTRY_(step_saggital_bounds, Eigen::Vector2d)                             \
  ARILES_TYPED_ENTRY_(step_coronal_bounds, Eigen::Vector2d)                              \
  ARILES_TYPED_ENTRY_(step_axial_bounds, Eigen::Vector2d)                                \
  ARILES_TYPED_ENTRY_(step_rotational_bounds, Eigen::Vector2d)                           \
  ARILES_TYPED_ENTRY_(static_walk_offset, Eigen::Vector2d)                               \
  ARILES_TYPED_ENTRY_(default_foot_separation, double)                                   \
  ARILES_TYPED_ENTRY_(rotational_foot_separation, double)                                \
  ARILES_TYPED_ENTRY_(integral_correction, double)                                       \
  ARILES_TYPED_ENTRY_(use_rate_limited_dcm, bool)                                        \
  ARILES_TYPED_ENTRY_(hybrid_factor, double)

#include ARILES_INITIALIZE


public:
  StaticWalkActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    acceptable_com_ds_reference_error_ = 0.015;
    acceptable_com_ss_reference_error_ = 0.015;
    preferred_com_velocity_ = 0.05;

    hybrid_factor_= 0.5;

    default_foot_separation_ = 0.18;
    rotational_foot_separation_ = default_foot_separation_;

    step_saggital_bounds_.setZero();
    step_coronal_bounds_ << default_foot_separation_, default_foot_separation_;
    step_axial_bounds_.setZero();
    step_rotational_bounds_.setZero();

    static_walk_offset_ << 0.0, 0.0;

    use_rate_limited_dcm_ = false;

    integral_correction_ = 0.0;

    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
  }

  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};



class StaticWalkAction : public WalkingActionBase
{
public:
  StaticWalkAction()
    : active_walk_state_(eVector3(0, 0, 0), eVector3(0, 0, 0), ros::Duration(0.0),
                         ros::Duration(0.0), SupporType::DS)
  {
  }

  StaticWalkAction(BController *bc);

  bool configure(ros::NodeHandle &nh, BController *bc,
                 const property_bag::PropertyBag &parameters) override;

  virtual ~StaticWalkAction();

  bool enterHook(const ros::Time &time);

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time);

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time);

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time);

protected:
  enum ReferencePoint
  {
    REFPOINT_UNDEFINED = 0,
    REFPOINT_FOOT_LEFT = 1,
    REFPOINT_FOOT_RIGHT = 2,
    REFPOINT_FOOT_MIDPOINT = 3
  };


  enum Stage
  {
    STAGE_UNDEFINED = 0,
    STAGE_DOUBLE_SUPPORT_COM_HOLD_REFPOINT = 1,
    STAGE_DOUBLE_SUPPORT_INIT_COM_TRAJECTORY = 2,
    STAGE_DOUBLE_SUPPORT_MOVE_COM = 3,
    STAGE_COM_HOLD_REFPOINT = 4,
    STAGE_SINGLE_SUPPORT_SWING_INITIALIZE = 5,
    STAGE_SINGLE_SUPPORT_SWING = 6
  };

  eMatrixHom getTargetPoseFromVel(const eMatrixHom &initial_stance_foot_pose);

  void initializeTorsoTraj(const eQuaternion &from, const eQuaternion &to);

  void initializeBaseTraj(const eQuaternion &from, const eQuaternion &to);

//  std::vector<std::pair<eMatrixHom, ros::Duration>> createFootTraj(
//      const eMatrixHom &initial_foot_pose, const eMatrixHom &target_foot_pose);

  void clampWaypoints(const eMatrixHom &support_leg_pose, const ros::Time &time);

  StaticWalkActionParameters parameters_;

  BController *bc_;

  Stage stage_;

  bool next_is_single_support_;

  ReferencePoint refpoint_;

  Side support_leg_;
  Side swing_leg_;

  ros::Time stage_duration_;
  ros::Time stage_timer_;
  ros::Time over_timer_;

  pal_robot_tools::MinJerkGenerator3D com_trajectory_;
  pal::pointmass::State desired_com_state_;

  pal_robot_tools::MinJerkGeneratorQuaternion base_orientation_trajectory_;
  pal::rigidbody::AngularState desired_base_angular_state_;

  pal_robot_tools::MinJerkGeneratorQuaternion torso_orientation_trajectory_;
  pal::rigidbody::AngularState desired_torso_angular_state_;

//  pal_robot_tools::PoseReferenceMinJerk foot_pose_trajectory_;

  pal_locomotion::SwingLegTrajectoryBasePtr foot_pose_trajectory_;

  pal_robot_tools::MinJerkGenerator force_distribution_interpolator_;

  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;

  // valid only during STAGE_DOUBLE_SUPPORT_MOVE_COM and STAGE_SINGLE_SUPPORT_SWING
  WalkingStepCommand active_walk_state_;

  ros::NodeHandle nh_;

  int actual_wypt_;
  std::vector<CartesianTrajectoryPoint> waypoints_;
  bool planned_footstep_;

  Eigen::Isometry3d target_foot_pose_;
  Eigen::Isometry3d prev_target_foot_pose_;
  double prev_COM_z_;

  Side prev_side_;

  unsigned int marray_index_;
  visualization_msgs::MarkerArray marray_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>> footstep_pub_;

  pal_statistics::RegistrationsRAII registered_variables_;
  unsigned int reg_stage_;
};

typedef boost::shared_ptr<StaticWalkAction> StaticWalkActionPtr;
}
