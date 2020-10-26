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

    getTrajectoryFromRosParam(nh, "com", com_traj_);
//    getTrajectoryFromRosParam(nh, "lfoot", lfoot_traj_);
//    getTrajectoryFromRosParam(nh, "rfoot", rfoot_traj_);

  return true;
}

bool CSVWALKINGActionPrev::getTrajectoryFromRosParam(const ros::NodeHandle &nh, const std::string & name, Trajectory & traj){
    std::vector<double> trajectory_t_;
    std::vector<double> trajectory_pos_x_;
    std::vector<double> trajectory_pos_y_;
    std::vector<double> trajectory_pos_z_;
    std::vector<double> trajectory_vel_x_;
    std::vector<double> trajectory_vel_y_;
    std::vector<double> trajectory_vel_z_;
    std::vector<double> trajectory_acc_x_;
    std::vector<double> trajectory_acc_y_;
    std::vector<double> trajectory_acc_z_;

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

    traj.pos.resize(3, std2eigen(trajectory_t_).size());
    traj.vel.resize(3, std2eigen(trajectory_t_).size());
    traj.acc.resize(3, std2eigen(trajectory_t_).size());

    traj.pos << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();
    traj.vel << std2eigen(trajectory_vel_x_).transpose(), std2eigen(trajectory_vel_y_).transpose(), std2eigen(trajectory_vel_z_).transpose();
    traj.acc << std2eigen(trajectory_acc_x_).transpose(), std2eigen(trajectory_acc_y_).transpose(), std2eigen(trajectory_acc_z_).transpose();
    ROS_INFO_STREAM(name + "_trajectory is generated.");

    std::vector<double> support_durations;
    key = "/support_durations";
    if (nh.getParam(key, support_durations)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    support_durations_ = std2eigen(support_durations);

    std::vector<double> support_end_times;
    key = "/support_end_times";
    if (nh.getParam(key, support_end_times)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    support_end_times_ = std2eigen(support_end_times);

    std::vector<double> support_indexes;
    key = "/support_indexes";
    if (nh.getParam(key, support_indexes)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    support_indexes_ = std2eigen(support_indexes);
    ROS_INFO_STREAM(support_indexes_);

    num_of_phases_ = support_indexes_.size();
    ROS_INFO_STREAM("num_of_phases_ ");
    ROS_INFO_STREAM(num_of_phases_);

}

//bool CSVWALKINGActionPrev::getCSVContactSequence(const ros::NodeHandle &nh, ContactSequenceFromCSV2 & cs){
//  std::string key = "/contact_sequence/t";
//  if (nh.getParam(key, trajectory_t_)){
//      ROS_INFO_STREAM("Successfully load " + key);
//  }else{
//      ROS_INFO_STREAM("Fail to load " + key);
//  }
//  cs.time = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());
//
//  key = "/contact_sequence/cs";
//  if (nh.getParam(key, trajectory_t_)){
//      ROS_INFO_STREAM("Successfully load " + key);
//  }else{
//      ROS_INFO_STREAM("Fail to load " + key);
//  }
//  cs.type = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());
//  ROS_INFO_STREAM( "Contact sequence is generated.");
//
//}

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
  begin_time_ = time;

//  control_time_ = internal_time_ + ros::Duration(com_traj_.time[com_traj_.time.size()-1]);
//  ds_time_ = internal_time_ + ros::Duration(1.0);
//  ss_time_ = internal_time_ + ros::Duration(2.0);
//  sss_time_ = internal_time_ + ros::Duration(3.0);
//  final_time_ = internal_time_ + ros::Duration(4.0);


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
  dt_ = bc_->getControllerDt();
    ROS_INFO_STREAM( "dt_:");
    ROS_INFO_STREAM( dt_.toSec());

    lf_pos_ = bc_->getActualFootPose(+Side::LEFT);
    rf_pos_ = bc_->getActualFootPose(+Side::RIGHT);



    ini_com_pos_ = bc_->getActualCOMPosition();
    ROS_INFO_STREAM( "lf_pos_:");
    ROS_INFO_STREAM( lf_pos_.translation());
    ROS_INFO_STREAM( "rf_pos_:");
    ROS_INFO_STREAM( rf_pos_.translation());
    ROS_INFO_STREAM( "ini_com_pos_:");
    ROS_INFO_STREAM( ini_com_pos_);


    lf_up_pos_ = lf_pos_;
    rf_up_pos_ = rf_pos_;
    lf_up_pos_.translation()(2) = 0.04;
    rf_up_pos_.translation()(2) = 0.04;

    ROS_INFO_STREAM( "lf_up_pos_:");
    ROS_INFO_STREAM( lf_up_pos_.translation());
    ROS_INFO_STREAM( "rf_up_pos_:");
    ROS_INFO_STREAM( rf_up_pos_.translation());

  return true;
}



bool CSVWALKINGActionPrev::cycleHook(const ros::Time &time)
{
//    ROS_INFO_STREAM( "CSVWALKINGActionPrev::cycleHook()");



  time_from_begin_ = time - begin_time_;
  cnt_ = int(time_from_begin_.toSec()/dt_.toSec());
  if (cnt_ > com_traj_.time.size()-1) {
      cnt_ = com_traj_.time.size()-1;
  }
//    ROS_INFO_STREAM( time_from_begin_.toSec());
//    ROS_INFO_STREAM( cnt_);

    int cur_support_index = 6666;
    for (int i=0; i < num_of_phases_; i++){
        if (time_from_begin_.toSec()>support_end_times_(i)){
            cur_support_index = support_indexes_(i);
            }
    }
//    ROS_INFO_STREAM( cur_support_index);

    eVector3 targetCOM_pos, targetCOM_vel, targetCOM_acc;
    targetCOM_pos = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    targetCOM_acc = com_traj_.acc.col(cnt_);



    if (cur_support_index == 0) // double support
    {
        bc_->setWeightDistribution(0.5);
        bc_->setActualSupportType(SupporType::DS);
        bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
        bc_->setSwingLegIDs({});
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


    }
    else if (cur_support_index == 1) // left support
    {
        bc_->setWeightDistribution(1.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::LEFT});
        bc_->setSwingLegIDs({Side::RIGHT});
        bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                                 lf_pos_,
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.));

        bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                                 rf_up_pos_,
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.),
                                 eVector3(0., 0., 0.));

    }
    else if (cur_support_index == -1) // right support
    {
        bc_->setWeightDistribution(0.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::RIGHT});
        bc_->setSwingLegIDs({Side::LEFT});
        bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                                 lf_up_pos_,
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

//    ROS_INFO_STREAM( targetCOM_pos);

//    bc_->setDesiredCOMPosition(targetCOM_pos);
//    bc_->setDesiredCOMVelocity(targetCOM_vel);
//    bc_->setDesiredCOMAcceleration(targetCOM_acc);

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


  
//  internal_time_ += bc_->getControllerDt();


    if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
        ROS_INFO_STREAM("Done");
    }

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
