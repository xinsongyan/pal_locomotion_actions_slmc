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

namespace pal_locomotion
{
class BalancingActionParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "BalancingActionParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(reference_type, std::string)                                       \
  ARILES_TYPED_ENTRY_(one_leg_support, std::string)                                      \
  ARILES_TYPED_ENTRY_(swing_leg_relative_position, Eigen::Vector3d)

#include ARILES_INITIALIZE


public:
  Side support_leg_;
  Side swing_leg_;


public:
  BalancingActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    reference_type_ = "ref_pose_minjerk_topic";
    one_leg_support_ = "";
    swing_leg_relative_position_.setZero();
  }

  void finalize()
  {
    if (false == one_leg_support_.empty())
    {
      if ("right" == one_leg_support_)
      {
        support_leg_ = Side::RIGHT;
        swing_leg_ = Side::LEFT;
      }
      else
      {
        if ("left" == one_leg_support_)
        {
          support_leg_ = Side::LEFT;
          swing_leg_ = Side::RIGHT;
        }
        else
        {
          PAL_THROW("Unknown support foot.");
        }
      }
    }
  }
};


class BalancingAction : public WalkingActionBase
{
public:
  BalancingAction()
  {
  }

  BalancingAction(BController *bc);

  bool configure(ros::NodeHandle &nh, BController *bc,
                 const property_bag::PropertyBag &parameters) override;

  virtual ~BalancingAction();

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
  enum Stage
  {
    STAGE_UNDEFINED = 0,
    STAGE_MOVE_COM_INITIALIZE = 1,
    STAGE_MOVE_COM = 2,
    STAGE_LIFT_LEG_INITIALIZE = 3,
    STAGE_LIFT_LEG = 4,
    STAGE_FOLLOW_REFERENCE_INITIALIZE = 5,
    STAGE_FOLLOW_REFERENCE = 6
  };


protected:
  BController *bc_;


  Stage stage_;
  ros::Duration stage_duration_;
  ros::Duration stage_timer_;

  pal_robot_tools::ReferenceAbstractPtr reference_;
  pal_robot_tools::PluginlibHelperPtr<pal_robot_tools::ReferenceAbstract> reference_loader_;

  BalancingActionParameters parameters_;

  Eigen::Isometry3d initial_pose_;

  pal::rbcomposite::PolynomialTrajectory com_trajectory_;
  pal::vectorstate::State com_position_sample_;

  pal::rbcomposite::PolynomialTrajectory foot_trajectory_;
  pal::vectorstate::State foot_position_sample_;
  Eigen::Isometry3d initial_foot_pose_;
  Eigen::Isometry3d desired_foot_pose_;

  Eigen::Isometry3d local_coordinate_frame_;
};

typedef boost::shared_ptr<BalancingAction> BalancingActionPtr;
}
