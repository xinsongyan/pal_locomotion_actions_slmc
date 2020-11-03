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

  force_distribution_interpolator_.reset(new MinJerkGenerator());
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
  initial_time_ = time;
  control_time_ = internal_time_ + ros::Duration(cs_.end_time(current_cs_));

  actual_com_= bc_->getActualCOMPosition();
 
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  bc_->setDesiredFootState(static_cast<int>(+Side::LEFT), actual_left_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT), actual_right_foot_pose,
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.),
                           eVector3(0., 0., 0.), eVector3(0., 0., 0.));

  bc_->setActualSide(+Side::RIGHT);

  cnt_ = 0;

  return true;
}

bool CSVWALKINGAction::cycleHook(const ros::Time &time)
{
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);
  eMatrixHom local_coordinate_frame = bc_->getActualFootPose(+Side::RIGHT); // Right foot (Swing)

  eVector3 targetCOM, targetCOM_vel;
  targetCOM = actual_com_;
  targetCOM_vel.setZero();

  double weight_distribution;
  double weight_distribution_d;
  double weight_distribution_dd;

  using namespace std;
  if (control_time_ < internal_time_ ){
      current_cs_ += 1;
      cs_change_ = true;
  }

  //update control time
  if (cs_change_ && current_cs_ < cs_.end_time.size())
    control_time_ = initial_time_ + ros::Duration(cs_.end_time(current_cs_));

  if (cnt_ < com_traj_.pos.cols()-1){ // whole control loop
    if (cs_.type(current_cs_) == 0){ // for initial dsp
        if (cs_change_){
            ROS_INFO_STREAM("DSP to remove contact");
            bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
            bc_->setSwingLegIDs({});
            if (current_cs_ == 0){
                if (cs_.type(current_cs_+1) == -1)
                    force_distribution_interpolator_->initialize({initial_time_, control_time_}, {0.5, 0.}, {0., 0.}, {0., 0.});
                else
                    force_distribution_interpolator_->initialize({initial_time_, control_time_}, {0.5, 1.}, {0., 0.}, {0., 0.});
            }
            else if (current_cs_ < cs_.end_time.size()) {
                ros::Time control_time_prev = initial_time_ + ros::Duration(cs_.end_time(current_cs_-1));
                if (cs_.type(current_cs_-1) == -1)
                    force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {0.0, 1.0}, {0., 0.}, {0., 0.});
                else
                    force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {1.0, 0.0}, {0., 0.}, {0., 0.});
            }
            else { // final dsp
                ros::Time control_time_prev = initial_time_ + ros::Duration(cs_.end_time(current_cs_-1));
                if (cs_.type(current_cs_+1) == -1)
                    force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {1.0, 0.5}, {0., 0.}, {0., 0.});
                else
                    force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {0.0, 0.5}, {0., 0.}, {0., 0.});
            }
            cs_change_ = false;
        }
        force_distribution_interpolator_->query(internal_time_, weight_distribution, weight_distribution_d, weight_distribution_dd);
        bc_->setWeightDistribution(weight_distribution);
    }
    else if (cs_.type(current_cs_) == 1){ // for ssp of LEFT support
        ros::Time control_time_prev = control_time_ - ros::Duration((cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1)));
        ros::Time control_time_half = control_time_prev + ros::Duration(  (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1))/2.0) ;

        if (cs_change_){
            ROS_INFO_STREAM("SSP with LEFT SUPPORT PHASE");
            bc_->setStanceLegIDs({Side::LEFT});
            bc_->setSwingLegIDs({Side::RIGHT});
            bc_->setWeightDistribution(1.0);
            
            initial_right_foot_pose_ = actual_right_foot_pose;
            l_swing_traj1_.initialize({control_time_prev, control_time_half}, {actual_right_foot_pose.translation().z(), actual_right_foot_pose.translation().z() + 0.05}, {0., 0.}, {0., 0.});
            l_swing_traj2_.initialize({control_time_half, control_time_}, {actual_right_foot_pose.translation().z()+0.05, actual_right_foot_pose.translation().z()}, {0., 0.}, {0., 0.});
            cs_change_ = false;
        }

        double height, height_d, height_dd;
        if (internal_time_ <= control_time_half){
            l_swing_traj1_.query(internal_time_, height, height_d, height_dd);
        }
        else{
            l_swing_traj2_.query(internal_time_, height, height_d, height_dd);
        }

        eMatrixHom target_foot_pose =  initial_right_foot_pose_;
        target_foot_pose.translation().z() = height;

         bc_->setDesiredFootState(
          static_cast<int>(Side::RIGHT), target_foot_pose,
          eVector3(0, 0, height_d),
          eVector3(0, 0, height_dd),
          eVector3(0, 0, 0),
          eVector3(0, 0, 0));
    }
    else if (cs_.type(current_cs_) == -1){ // for ssp of Right support
        ros::Time control_time_prev = control_time_ - ros::Duration((cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1)));
        ros::Time control_time_half = control_time_prev + ros::Duration(  (cs_.end_time(current_cs_) - cs_.end_time(current_cs_-1))/2.0) ;

        if (cs_change_){
            ROS_INFO_STREAM("SSP with RIGHT SUPPORT PHASE");
            bc_->setStanceLegIDs({Side::RIGHT});
            bc_->setSwingLegIDs({Side::LEFT});
            bc_->setWeightDistribution(0.0);

            initial_left_foot_pose_ = actual_left_foot_pose;
            r_swing_traj1_.initialize({control_time_prev, control_time_half}, {actual_left_foot_pose.translation().z(), actual_left_foot_pose.translation().z() + 0.05}, {0., 0.}, {0., 0.});
            r_swing_traj2_.initialize({control_time_half, control_time_}, {actual_left_foot_pose.translation().z()+0.05, actual_left_foot_pose.translation().z()}, {0., 0.}, {0., 0.});
            cs_change_ = false;
        }

        double height, height_d, height_dd;
        if (internal_time_ <= control_time_half){
            r_swing_traj1_.query(internal_time_, height, height_d, height_dd);
        }
        else{
            r_swing_traj2_.query(internal_time_, height, height_d, height_dd);
        }

        eMatrixHom target_foot_pose =  initial_left_foot_pose_;
        target_foot_pose.translation().z() = height;

         bc_->setDesiredFootState(
          static_cast<int>(Side::LEFT), target_foot_pose,
          eVector3(0, 0, height_d),
          eVector3(0, 0, height_dd),
          eVector3(0, 0, 0),
          eVector3(0, 0, 0));
    }
    // else { // for final dsp
    //     if (cs_change_){
    //         ROS_INFO_STREAM("DSP to recover balance");
    //         bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
    //         bc_->setSwingLegIDs({});

    //         ros::Time control_time_prev = initial_time_ + ros::Duration(cs_.end_time(current_cs_-1));
    //         if (cs_.type(current_cs_-1) == -1)
    //             force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {0.0, 0.5}, {0., 0.}, {0., 0.});
    //         else
    //             force_distribution_interpolator_->initialize({control_time_prev, control_time_}, {1.0, 0.5}, {0., 0.}, {0., 0.});
    //         cs_change_ = false;
    //     }
    //     force_distribution_interpolator_->query(internal_time_, weight_distribution, weight_distribution_d, weight_distribution_dd);
    //     bc_->setWeightDistribution(weight_distribution);
    // }
    targetCOM =  actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(cnt_);
    cnt_++;
  }
  else{
    targetCOM = actual_com_ + com_traj_.pos.col(cnt_) - com_traj_.pos.col(0);
    targetCOM_vel.setZero();
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
