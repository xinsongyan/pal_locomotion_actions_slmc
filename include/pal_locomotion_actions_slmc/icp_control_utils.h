/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _ICP_CONTROL_UTILS_
#define _ICP_CONTROL_UTILS_

#include <pal_locomotion/biped_controller.h>

namespace pal_locomotion
{
void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped);

void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm_,
             const eVector3 &actualCOM, const eVector2 &actualCOM_vel,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped);
}

#endif
