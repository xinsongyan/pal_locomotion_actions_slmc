#include <pal_locomotion_actions_slmc/csv_com_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>

//ros::init("param_loader");


rapidcsv::Document csv_com_reader_("/home/xin/catkin_ws/src/pal_locomotion_actions_slmc/trajectory/com_trajectory.csv");
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


  pb.getPropertyValue<std::string>("filename", parameters_.filename_,
                              property_bag::RetrievalHandling::THROW);


  getComTrajectory(nh);


  return true;
}

bool CSVCOMAction::getComTrajectory(ros::NodeHandle &nh){

    std::string key = "/com_trajectory/t";
    if (nh.getParam(key, com_trajectory_t_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

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


}



bool CSVCOMAction::enterHook(const ros::Time &time)
{
  support_type_ = bc_->getActualSupportType();

  int csv_size = csv_com_reader_.GetRowCount();
  internal_time_ = time;
  control_time_ = internal_time_ + ros::Duration(csv_com_reader_.GetRow<float>(csv_size-1)[0]);
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
    local_coordinate_frame =
        interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
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

//  if (cnt_ < csv_com_reader_.GetRowCount()){
//    for (int i=1 ; i<4; i++) {
//          targetCOM(i-1) = actual_com_(i-1)+ csv_com_reader_.GetRow<float>(cnt_)[i] - csv_com_reader_.GetRow<float>(0)[i];
//          targetCOM_vel(i-1) =  csv_com_reader_.GetRow<float>(cnt_)[i+3];
//    }
//    cnt_ += 1;
//  }
//  else
//  {
//    for (int i=1 ; i<4; i++) {
//     targetCOM(i-1) = actual_com_(i-1) + csv_com_reader_.GetRow<float>(csv_com_reader_.GetRowCount()-1)[i] - csv_com_reader_.GetRow<float>(0)[i];
//     targetCOM_vel(i-1) =  csv_com_reader_.GetRow<float>(csv_com_reader_.GetRowCount()-1)[i+3];
//    }
//  }


    if (cnt_ < csv_com_reader_.GetRowCount()){
        targetCOM(0) = actual_com_[0] + com_trajectory_pos_x_[cnt_] - com_trajectory_pos_x_[0];
        targetCOM(1) = actual_com_[1] + com_trajectory_pos_y_[cnt_] - com_trajectory_pos_y_[0];
        targetCOM(2) = actual_com_[2] + com_trajectory_pos_z_[cnt_] - com_trajectory_pos_z_[0];
        targetCOM_vel(0) = com_trajectory_vel_x_[cnt_];
        targetCOM_vel(1) = com_trajectory_vel_y_[cnt_];
        targetCOM_vel(2) = com_trajectory_vel_z_[cnt_];
        cnt_ += 1;
    }
    else
    {
        targetCOM(0) = actual_com_[0] + com_trajectory_pos_x_[cnt_] - com_trajectory_pos_x_[0];
        targetCOM(1) = actual_com_[1] + com_trajectory_pos_y_[cnt_] - com_trajectory_pos_y_[0];
        targetCOM(2) = actual_com_[2] + com_trajectory_pos_z_[cnt_] - com_trajectory_pos_z_[0];
        targetCOM_vel(0) = com_trajectory_vel_x_[cnt_];
        targetCOM_vel(1) = com_trajectory_vel_y_[cnt_];
        targetCOM_vel(2) = com_trajectory_vel_z_[cnt_];
    }


  if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
    ROS_INFO_STREAM("Done");
    std::cout << cnt_ <<std::endl;
  }

  eVector2 global_target_cop = targetCOM.head(2);

  control(bc_, rate_limiter_, targetCOM, targetCOM_vel, global_target_cop,
          parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
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
