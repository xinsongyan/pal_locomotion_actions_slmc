/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _MOVE_ICP_DS_ACTION_
#define _MOVE_ICP_DS_ACTION_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>

namespace pal_locomotion
{
enum class BalanceSupporType
{
  DS,
  LEFT,
  RIGHT
};

struct MoveICPDSActionParameters
{
  MoveICPDSActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    switch_duration_ = 5.0;
    use_rate_limited_dcm_ = false;

    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
  }

  double switch_duration_;
  bool use_rate_limited_dcm_;

  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};

class MoveICPDSAction : public WalkingActionBase
{
public:
  MoveICPDSAction();

  virtual ~MoveICPDSAction();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

private:
  bool configure_interpolator_;
  bool initial_interpolation_;

  BController *bc_;
  BalanceSupporType support_type_;
  pal_robot_tools::MinJerkGenerator2DPtr icp_interpolator_;

  ros::Time internal_time_;
  ros::Time switch_time_;

  MoveICPDSActionParameters parameters_;

  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  ddynamic_reconfigure::DDynamicReconfigurePtr ddr_;
};
}

#endif
