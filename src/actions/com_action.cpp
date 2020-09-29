#include <pal_locomotion_actions_slmc/com_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;

namespace pal_locomotion
{
COMAction::COMAction()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{
}

COMAction::~COMAction()
{
}

bool COMAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
  bc_ = bController;
  

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  icp_interpolator_.reset(new MinJerkGenerator2D());

  pb.getPropertyValue<double>("duration", parameters_.duration_,
                              property_bag::RetrievalHandling::THROW);
  pb.getPropertyValue<eVector3>("target_com_position", parameters_.target_com_position_,
                                property_bag::RetrievalHandling::THROW);

  control_time_ = internal_time_ + ros::Duration(parameters_.duration_);
  return true;
}

bool COMAction::enterHook(const ros::Time &time)
{
  support_type_ = bc_->getActualSupportType();
  internal_time_ = time;
  control_time_ = internal_time_ + ros::Duration(parameters_.duration_);
  actual_com_.translation() = bc_->getActualCOMPosition();
  desired_com_ = actual_com_;
  desired_com_.translation() += parameters_.target_com_position_;
  return true;
}

bool COMAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame;
 

  // define local frame
  if (support_type_ == +SupporType::DS)
  {
    local_coordinate_frame =
        interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
  }
  else
  {
    PAL_THROW_DEFAULT("not supported for Single Support");
  }


  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);


  if (configure_interpolator_)
  {
    if (initial_interpolation_)
    {
      icp_interpolator_->initialize(
          { internal_time_, control_time_ },
          { (local_coordinate_frame.inverse() *
             interpolateBetweenTransforms(actual_com_, desired_com_))
                .translation().head(2),
            eVector2::Zero() },
          { eVector2::Zero(), eVector2::Zero() }, { eVector2::Zero(), eVector2::Zero() });

      initial_interpolation_ = false;
    }
    else
    {
      PAL_THROW_DEFAULT("not supported");
    }
    configure_interpolator_ = false;
  }

  eVector2 local_target_dcm;
  eVector2 local_target_dcm_vel;
  eVector2 local_target_dcm_acc;

  if (internal_time_ < control_time_){
    icp_interpolator_->query(internal_time_, local_target_dcm, local_target_dcm_vel,
                            local_target_dcm_acc);
  }
  else if (internal_time_ >= control_time_)
  {
    icp_interpolator_->query(control_time_, local_target_dcm, local_target_dcm_vel,
                            local_target_dcm_acc);
  }

  if (internal_time_ == control_time_){
    ROS_INFO_STREAM("Done");
  }

  eVector2 global_target_dcm = local_coordinate_frame_2d * local_target_dcm;
  eVector2 global_target_cop = global_target_dcm;
  eVector2 global_target_dcm_vel = local_coordinate_frame_2d.rotation() * local_target_dcm_vel;
  double desired_z = local_coordinate_frame.translation().z() + bc_->getParameters()->z_height_ ;
  double desired_z_d = 0.;

  eVector3 targetCOM;
  targetCOM << global_target_dcm.x(), global_target_dcm.y(), desired_z;
  eVector3 targetCOM_vel;
  targetCOM_vel << global_target_dcm_vel.x(), global_target_dcm_vel.y(), desired_z_d;

  control(bc_, rate_limiter_, targetCOM, targetCOM_vel, global_target_cop,
          parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
          targetCOP_unclamped_);

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  
  internal_time_ += bc_->getControllerDt();


  return true;
}

bool COMAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool COMAction::endHook(const ros::Time &time)
{
  return true;
}
}