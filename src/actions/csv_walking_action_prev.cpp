#include <pal_locomotion_actions_slmc/csv_walking_action_prev.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>



namespace pal_locomotion
{
CSVWALKINGActionPrev::CSVWALKINGActionPrev()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{

}

CSVWALKINGActionPrev::~CSVWALKINGActionPrev()
{
}

bool CSVWALKINGActionPrev::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
  bc_ = bController;
  

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  getCSVTrajectory(nh, "com", com_traj_);
  getCSVTrajectory(nh, "lfoot", lfoot_traj_);
  getCSVTrajectory(nh, "rfoot", rfoot_traj_);
  getCSVContactSequence(nh, cs_);
  return true;
}

bool CSVWALKINGActionPrev::getCSVTrajectory(const ros::NodeHandle &nh, const std::string & name, TrajectoryFromCSV2 & traj){
    std::string key = "/" + name + "_trajectory/t";
    if (nh.getParam(key, trajectory_t_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    traj.time = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());

    key = "/" + name + "_trajectory/pos/x";
    if (nh.getParam(key, trajectory_pos_x_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/pos/y";
    if (nh.getParam(key, trajectory_pos_y_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/pos/z";
    if (nh.getParam(key, trajectory_pos_z_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/vel/x";
    if (nh.getParam(key, trajectory_vel_x_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/vel/y";
    if (nh.getParam(key, trajectory_vel_y_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/vel/z";
    if (nh.getParam(key, trajectory_vel_z_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/acc/x";
    if (nh.getParam(key, trajectory_acc_x_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/acc/y";
    if (nh.getParam(key, trajectory_acc_y_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    key = "/" + name + "_trajectory/acc/z";
    if (nh.getParam(key, trajectory_acc_z_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    traj.pos.resize(3, std2eigen(trajectory_pos_x_).size());
    traj.vel.resize(3, traj.pos.cols());
    traj.acc.resize(3, traj.pos.cols());

    traj.pos << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();
    traj.vel << std2eigen(trajectory_vel_x_).transpose(), std2eigen(trajectory_vel_y_).transpose(), std2eigen(trajectory_vel_z_).transpose();
    traj.acc << std2eigen(trajectory_acc_x_).transpose(), std2eigen(trajectory_acc_y_).transpose(), std2eigen(trajectory_acc_z_).transpose();
    ROS_INFO_STREAM(name + "_trajectory is generated.");
}

bool CSVWALKINGActionPrev::getCSVContactSequence(const ros::NodeHandle &nh, ContactSequenceFromCSV2 & cs){
  std::string key = "/contact_sequence/t";
  if (nh.getParam(key, trajectory_t_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }
  cs.time = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());

  key = "/contact_sequence/cs";
  if (nh.getParam(key, trajectory_t_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }
  cs.type = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());
  ROS_INFO_STREAM( "Contact sequence is generated.");

}
bool CSVWALKINGActionPrev::enterHook(const ros::Time &time)
{

  bc_->setHybridControlFactor("leg_left_1_joint", 1.);
  bc_->setHybridControlFactor("leg_left_2_joint", 1.);
  bc_->setHybridControlFactor("leg_left_3_joint", 1.);
  bc_->setHybridControlFactor("leg_left_4_joint", 1.);
  bc_->setHybridControlFactor("leg_left_5_joint", 1.);
  bc_->setHybridControlFactor("leg_left_6_joint", 1.);
  bc_->setHybridControlFactor("leg_right_1_joint", 1.);
  bc_->setHybridControlFactor("leg_right_2_joint", 1.);
  bc_->setHybridControlFactor("leg_right_3_joint", 1.);
  bc_->setHybridControlFactor("leg_right_4_joint", 1.);
  bc_->setHybridControlFactor("leg_right_5_joint", 1.);
  bc_->setHybridControlFactor("leg_right_6_joint", 1.);

  support_type_ = bc_->getActualSupportType();

  internal_time_ = time;

  control_time_ = internal_time_ + ros::Duration(com_traj_.time[com_traj_.time.size()-1]);
  ds_time_ = internal_time_ + ros::Duration(1.0);
  ss_time_ = internal_time_ + ros::Duration(2.0);
  sss_time_ = internal_time_ + ros::Duration(3.0);
  final_time_ = internal_time_ + ros::Duration(4.0);


  actual_com_= bc_->getActualCOMPosition();
 
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT), actual_left_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT), actual_right_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  // bc_->setActualSide(+Side::LEFT);

  cnt_ = 0;

  return true;
}

bool CSVWALKINGActionPrev::cycleHook(const ros::Time &time)
{
  eVector3 targetCOM_pos, targetCOM_vel, targetCOM_acc;
  targetCOM_pos = actual_com_;
  targetCOM_vel.setZero();
  
  if (cnt_== 0){
    lf_pos_ = bc_->getActualFootPose(+Side::LEFT);
    rf_pos_ = bc_->getActualFootPose(+Side::RIGHT);
  }
  if (time < ds_time_)
  {
    targetCOM_pos = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);

//    bc_->setWeightDistribution(0.5 + 0.5 * (cnt_) / 500.0); // Left Side?
      bc_->setWeightDistribution(0.5); // Left Side?
    cnt_ += 1;
  }
  else if (time < ss_time_) {
    
    bc_->setStanceLegIDs({Side::LEFT});
    bc_->setSwingLegIDs({Side::RIGHT});
    targetCOM_pos = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);

    bc_->setWeightDistribution(1.0); // Left Side?
    cnt_ += 1;
  }
  else if (time < sss_time_) {
    bc_->setStanceLegIDs({Side::RIGHT});
    bc_->setSwingLegIDs({Side::LEFT});

    targetCOM_pos = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);

    bc_->setWeightDistribution(0); // Right Side?
    cnt_ += 1;
  }
  else if (cnt_ < com_traj_.pos.cols()-1){

    bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
    bc_->setSwingLegIDs({});
    targetCOM_pos = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);
//    bc_->setWeightDistribution( 0.5 * (cnt_ - 1500) / 500.0 );
      bc_->setWeightDistribution( 0.5 );
    cnt_ += 1;
  }
  else{
  bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
  bc_->setSwingLegIDs({});
    targetCOM_pos = actual_com_;
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);
    targetCOM_vel.setZero();
    targetCOM_acc.setZero();
    bc_->setWeightDistribution(0.5); // Left Side?


  }

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                           lf_pos_,
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                           rf_pos_,
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.));
  
  if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
    ROS_INFO_STREAM("Done");
  }

  eVector2 global_target_cop = targetCOM_pos.head(2);

  control(bc_,
          rate_limiter_,
          targetCOM_pos,
          targetCOM_vel,
          global_target_cop,
          parameters_.use_rate_limited_dcm_,
          targetCOP_rate_limited_unclamped_,
          targetCOP_unclamped_);


//    bc_->setDesiredCOMPosition(targetCOM_pos);
//    bc_->setDesiredCOMVelocity(targetCOM_vel);
//    bc_->setDesiredCOMAcceleration(targetCOM_acc);
//
//    bc_->setDesiredICP(eVector3(targetCOM_pos.x(), targetCOM_pos.y(), 0.));
//    bc_->setDesiredCOPReference(eVector3(targetCOM_pos.x(), targetCOM_pos.y(), 0.));
//
//
//    eVector2 targetCoM_pos2D(targetCOM_pos.x(), targetCOM_pos.y());
//    eVector2 targetCoM_vel2D(targetCOM_vel.x(), targetCOM_vel.y());
//    double w = sqrt(bc_->getParameters()->gravity_ / bc_->getParameters()->z_height_);
//    Eigen::Vector2d actualDCM = targetCoM_pos2D + targetCoM_vel2D / w;
//    bc_->setActualICP(eVector3(actualDCM.x(), actualDCM.y(), 0.));


  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));


  
  internal_time_ += bc_->getControllerDt();

  return true;
}

bool CSVWALKINGActionPrev::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool CSVWALKINGActionPrev::endHook(const ros::Time &time)
{
  return true;
}

Eigen::VectorXd CSVWALKINGActionPrev::std2eigen(std::vector<double> std_vec){
    Eigen::VectorXd eigen_vec = Eigen::Map<Eigen::VectorXd>(std_vec.data(), std_vec.size());
    return eigen_vec;
}
}
