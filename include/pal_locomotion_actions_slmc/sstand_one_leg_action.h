/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _SSTAND_ONE_LEG_ACTION_
#define _SSTAND_ONE_LEG_ACTION_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk_topic.h>

namespace pal_locomotion
{
struct SStandOneLegActionParameters : ariles::ConfigurableBase
{
  SStandOneLegActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
    use_rate_limited_dcm_ = false;
  }

#define ARILES_SECTION_ID "SStandOneLegActionParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(use_rate_limited_dcm)                                                    \
  ARILES_ENTRY_(ds_duration)                                                             \
  ARILES_ENTRY_(swing_leg_duration)                                                      \
  ARILES_ENTRY_(target_swing_leg_orientation)                                            \
  ARILES_ENTRY_(target_swing_leg_position)                                               \
  ARILES_ENTRY_(side)
#include ARILES_INITIALIZE

  bool use_rate_limited_dcm_;
  double ds_duration_;
  std::vector<double> swing_leg_duration_;
  std::vector<eQuaternion> target_swing_leg_orientation_;
  std::vector<eVector3> target_swing_leg_position_;

  Side side_;

  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};

class SStandOneLegAction : public WalkingActionBase
{
public:
  SStandOneLegAction();

  virtual ~SStandOneLegAction();

  bool configure(ros::NodeHandle &nh, BController *bc, const property_bag::PropertyBag &pb) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

private:
  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;

  BController *bc_;

  SStandOneLegActionParameters parameters_;

  ros::Time internal_time_;
  ros::Time ds_time_;
  ros::Time ss_time_;

  pal_robot_tools::MinJerkGeneratorPtr force_distribution_interpolator_;

  pal_robot_tools::MinJerkGenerator2DPtr icp_interpolator_;
  pal_robot_tools::PoseReferenceMinJerkTopicPtr swing_leg_interpolator_;

  //  eMatrixHom swing_leg_interpolator_initial_reference_frame_;

  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  bool swing_leg_configured_;

  int swing_leg_pose_index_;

  pal_robot_tools::MinJerkGenerator height_trajectory_;

  eVector2 offset_;

};
}


#endif
