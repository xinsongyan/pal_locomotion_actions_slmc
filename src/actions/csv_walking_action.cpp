#include <pal_locomotion_actions_slmc/csv_walking_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>



namespace pal_locomotion
{
CSVWALKINGAction::CSVWALKINGAction()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{

}

CSVWALKINGAction::~CSVWALKINGAction()
{
}

bool CSVWALKINGAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
  bc_ = bController;
  

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));

  getCSVTrajectory(nh, "com", com_traj_);
  getCSVContactSequence(nh, cs_);

  current_cs_ = 0;
  cs_change_ = true;
  return true;
}

bool CSVWALKINGAction::getCSVTrajectory(const ros::NodeHandle &nh, const std::string & name, TrajectoryFromCSV & traj){
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

    traj.pos.resize(3, std2eigen(trajectory_pos_x_).size());
    traj.vel.resize(3, traj.pos.cols());

    traj.pos << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();
    traj.vel << std2eigen(trajectory_vel_x_).transpose(), std2eigen(trajectory_vel_y_).transpose(), std2eigen(trajectory_vel_z_).transpose();
    ROS_INFO_STREAM(name + "_trajectory is generated.");
}

bool CSVWALKINGAction::getCSVContactSequence(const ros::NodeHandle &nh, ContactSequenceFromCSV & cs){
  std::string key = "/contact_sequence/t_end";
  if (nh.getParam(key, trajectory_t_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }
  cs.end_time = Eigen::Map<Eigen::VectorXd>(trajectory_t_.data(), trajectory_t_.size());

  key = "/contact_sequence/oMi_R/x";
  if (nh.getParam(key, trajectory_pos_x_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMi_R/y";
  if (nh.getParam(key, trajectory_pos_y_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMi_R/z";
  if (nh.getParam(key, trajectory_pos_z_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  cs.oMi_R.resize(3, std2eigen(trajectory_pos_x_).size());
  cs.oMi_R << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();

  key = "/contact_sequence/oMi_L/x";
  if (nh.getParam(key, trajectory_pos_x_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMi_L/y";
  if (nh.getParam(key, trajectory_pos_y_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMi_L/z";
  if (nh.getParam(key, trajectory_pos_z_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  cs.oMi_L.resize(3, std2eigen(trajectory_pos_x_).size());
  cs.oMi_L << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();

  key = "/contact_sequence/oMf/x";
  if (nh.getParam(key, trajectory_pos_x_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMf/y";
  if (nh.getParam(key, trajectory_pos_y_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  key = "/contact_sequence/oMf/z";
  if (nh.getParam(key, trajectory_pos_z_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }

  cs.oMf.resize(3, std2eigen(trajectory_pos_x_).size());
  cs.oMf << std2eigen(trajectory_pos_x_).transpose(), std2eigen(trajectory_pos_y_).transpose(), std2eigen(trajectory_pos_z_).transpose();

  key = "/contact_sequence/type";
  if (nh.getParam(key, contact_type_)){
      ROS_INFO_STREAM("Successfully load " + key);
  }else{
      ROS_INFO_STREAM("Fail to load " + key);
  }
  cs.type = Eigen::Map<Eigen::VectorXi>(contact_type_.data(), contact_type_.size());
  ROS_INFO_STREAM( "Contact sequence is generated.");
}
bool CSVWALKINGAction::enterHook(const ros::Time &time)
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

  actual_com_= bc_->getActualCOMPosition();
 
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT), actual_left_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT), actual_right_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setActualSide(+Side::LEFT);

  cnt_ = 0;

  return true;
}

bool CSVWALKINGAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
  eMatrixHom local_coordinate_frame = bc_->getActualFootPose(+Side::LEFT); // Right foot (Swing)

  eVector3 targetCOM, targetCOM_vel;
  targetCOM = actual_com_;
  targetCOM_vel.setZero();
  
  ros::Duration cs_duration = time - internal_time_;
  double cs_time = cs_duration.toSec() + cs_duration.toNSec() * 1e-9;
  if (cnt_ < com_traj_.pos.cols()-1){
    if (cs_.type(current_cs_) == 0){
      if (current_cs_ == 0){
        bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});

        if (cs_.type(current_cs_+1) == -1)
          bc_->setWeightDistribution(0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_))   );
        else
          bc_->setWeightDistribution(1.0 - 0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_))   );
      }
      else{
        if (cs_.type(current_cs_+1) == -1)
         bc_->setWeightDistribution(0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1) )   );
        else
         bc_->setWeightDistribution(1.0 - 0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1) )   );
      }
    } 
    else if (cs_.type(current_cs_) == 1){
      bc_->setStanceLegIDs({Side::LEFT});
      bc_->setSwingLegIDs({Side::RIGHT});
      bc_->setWeightDistribution(1.);
    }
    else if (cs_.type(current_cs_) == -1){
      bc_->setStanceLegIDs({Side::RIGHT});
      bc_->setSwingLegIDs({Side::LEFT});
      bc_->setWeightDistribution(0.);
    }
    else{
      bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});

      if (cs_.type(current_cs_-1) == -1)
       bc_->setWeightDistribution(0.5 - 0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1) )  );
      else
       bc_->setWeightDistribution(0.5 + 0.5 * (cs_.end_time(current_cs_) - cs_time ) / (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1) )  );
    }


    targetCOM =  actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    cnt_ += 1;
  }
  else{
    targetCOM = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel.setZero();
  }

  if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
    ROS_INFO_STREAM("Done");
  }

  eVector2 global_target_cop = targetCOM.head(2);

  control(bc_,
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

bool CSVWALKINGAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool CSVWALKINGAction::endHook(const ros::Time &time)
{
  return true;
}

Eigen::VectorXd CSVWALKINGAction::std2eigen(std::vector<double> std_vec){
    Eigen::VectorXd eigen_vec = Eigen::Map<Eigen::VectorXd>(std_vec.data(), std_vec.size());
    return eigen_vec;
}
}
