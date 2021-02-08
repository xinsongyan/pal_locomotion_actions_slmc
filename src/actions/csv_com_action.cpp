#include <pal_locomotion_actions_slmc/csv_com_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>



namespace pal_locomotion
{
CSVCOMAction::CSVCOMAction()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{

}

CSVCOMAction::~CSVCOMAction()
{
}

bool CSVCOMAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
  bc_ = bController;
  

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));


//  pb.getPropertyValue<std::string>("filename", parameters_.filename_,
//                              property_bag::RetrievalHandling::THROW);


  getComTrajectory(nh);


  return true;
}

void CSVCOMAction::getComTrajectory(ros::NodeHandle &nh){

    std::string key = "/com_trajectory/t";
    if (nh.getParam(key, com_trajectory_t_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    com_t_ = Eigen::Map<Eigen::VectorXd>(com_trajectory_t_.data(), com_trajectory_t_.size());

    key = "/com_trajectory/pos/x";
    if (nh.getParam(key, com_trajectory_pos_x_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/com_trajectory/pos/y";
    if (nh.getParam(key, com_trajectory_pos_y_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/com_trajectory/pos/z";
    if (nh.getParam(key, com_trajectory_pos_z_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/com_trajectory/vel/x";
    if (nh.getParam(key, com_trajectory_vel_x_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/com_trajectory/vel/y";
    if (nh.getParam(key, com_trajectory_vel_y_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/com_trajectory/vel/z";
    if (nh.getParam(key, com_trajectory_vel_z_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    Eigen::VectorXd com_pos_x_ = std2eigen(com_trajectory_pos_x_);
    Eigen::VectorXd com_pos_y_ = std2eigen(com_trajectory_pos_y_);
    Eigen::VectorXd com_pos_z_ = std2eigen(com_trajectory_pos_z_);

    com_pos_.resize(3, com_pos_x_.size());
    com_pos_ << com_pos_x_.transpose(), com_pos_y_.transpose(), com_pos_z_.transpose();
    ROS_INFO_STREAM("eigen com_pos generated.");

    Eigen::VectorXd com_vel_x_ = std2eigen(com_trajectory_vel_x_);
    Eigen::VectorXd com_vel_y_ = std2eigen(com_trajectory_vel_y_);
    Eigen::VectorXd com_vel_z_ = std2eigen(com_trajectory_vel_z_);

    com_vel_.resize(3, com_vel_x_.size());
    com_vel_ << com_vel_x_.transpose(), com_vel_y_.transpose(), com_vel_z_.transpose();
    ROS_INFO_STREAM("eigen com_vel generated.");
}

Eigen::VectorXd CSVCOMAction::std2eigen(std::vector<double> std_vec){
    Eigen::VectorXd eigen_vec = Eigen::Map<Eigen::VectorXd>(std_vec.data(), std_vec.size());
    return eigen_vec;
}


bool CSVCOMAction::enterHook(const ros::Time &time)
{
  support_type_ = bc_->getActualSupportType();

//  int csv_size = csv_com_reader_.GetRowCount();
  internal_time_ = time;

  control_time_ = internal_time_ + ros::Duration(com_t_[com_t_.size()-1]);
  actual_com_= bc_->getActualCOMPosition();
  cnt_ = 0;

  return true;
}

bool CSVCOMAction::cycleHook(const ros::Time &time)
{
 
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame;
 

  // define local frame
  if (support_type_ == +SupporType::DS)
  {
    local_coordinate_frame = interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
  }
  else
  {
    PAL_THROW_DEFAULT("not supported for Single Support");
  }


  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);

  eVector3 targetCOM, targetCOM_vel;
  targetCOM = actual_com_;
  targetCOM_vel.setZero();


    if (cnt_ < com_pos_.cols()-1){
        targetCOM = actual_com_ + com_pos_.col(cnt_) - com_pos_.col(0);
        targetCOM_vel = com_vel_.col(cnt_);
        cnt_ += 1;
    }
    else
    {
        targetCOM = actual_com_ + com_pos_.col(cnt_) - com_pos_.col(0);
        targetCOM_vel = com_vel_.col(cnt_);
    }


  if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
    ROS_INFO_STREAM("Done");
  }
 
  eVector2 global_target_cop = targetCOM.head(2);

  icp_control(bc_,
          rate_limiter_,
          targetCOM,
          targetCOM_vel,
          global_target_cop,
          parameters_.use_rate_limited_dcm_,
          targetCOP_rate_limited_unclamped_,
          targetCOP_unclamped_);

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));

  internal_time_ += bc_->getControllerDt();

  return true;
}

bool CSVCOMAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool CSVCOMAction::endHook(const ros::Time &time)
{
  return true;
}
}
