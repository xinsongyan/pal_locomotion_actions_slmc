/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _WALKING_ACTION_BASE_
#define _WALKING_ACTION_BASE_

#include <pal_utils/sm/action.h>
#include <pal_locomotion/biped_controller.h>
#include <property_bag/property_bag.h>

namespace pal_locomotion
{
class WalkingActionBase : public pal_robot_tools::Action
{
public:
  virtual ~WalkingActionBase()
  {
  }

  virtual bool configure(ros::NodeHandle& nh, BController* bController,
                         const property_bag::PropertyBag& parameters) = 0;
};

typedef boost::shared_ptr<WalkingActionBase> WalkingActionBasePtr;
}

#endif
