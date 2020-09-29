/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _BALANCE_DS_ACTION_
#define _BALANCE_DS_ACTION_

#include <pal_locomotion/biped_controller.h>
#include <pal_locomotion/polynomial.h>
#include <deque>
#include <pal_locomotion/state_machine/walking_action_base.h>
#include <math_utils/first_order_low_pass_filter.h>
#include <math_utils/high_pass_rate_limiter.h>
#include <math_utils/high_pass_rate_limiter.h>

namespace pal_locomotion
{
struct BalanceDsActionParameters : public ariles::ConfigurableBase
{
  BalanceDsActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    target_filter_cutoff_ = 100.;
    use_filtered_target_ = true;
    icp_control_ = true;

    use_rate_limited_dcm_ = false;

    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
  }

#define ARILES_SECTION_ID "BalanceDsActionParameters"
#define ARILES_CONSTRUCTOR BalanceDsActionParameters
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(target_filter_cutoff)                                                    \
  ARILES_ENTRY_(use_filtered_target)                                                     \
  ARILES_ENTRY_(icp_control)                                                             \
  ARILES_ENTRY_(use_rate_limited_dcm)
#include ARILES_INITIALIZE

  double target_filter_cutoff_;
  bool use_filtered_target_;
  bool icp_control_;

  bool use_rate_limited_dcm_;
  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};

class BalanceDsAction : public WalkingActionBase
{
public:
  BalanceDsAction()
  {
  }

  BalanceDsAction(ros::NodeHandle &nh, const BalanceDsActionParameters &action_parameters,
                  BController *bController);

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  virtual ~BalanceDsAction();

  bool enterHook(const ros::Time &time) override;

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time) override;

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time) override;

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time) override;

private:

  void visualize(const ros::Time &time);

  BController *bc_;

  BalanceDsActionParameters action_parameters_;
  math_utils::FirstOrderLowPassFilterVector3dPtr target_filter_;

  eVector3 initial_target_;

  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;

  Eigen::Vector2d targetCOP_unclamped_;
  Eigen::Vector2d targetCOP_rate_limited_unclamped_;

  pal_statistics::RegistrationsRAII registered_variables_;
};

typedef boost::shared_ptr<BalanceDsAction> BalanceDsActionPtr;
}

#endif
