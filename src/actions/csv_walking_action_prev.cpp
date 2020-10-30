#include <pal_locomotion_actions_slmc/csv_walking_action_prev.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>


using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>

namespace pal_locomotion
{
CSVWALKINGActionPrev::CSVWALKINGActionPrev()
  : internal_time_(ros::Time(0)),
    lfoot_swing_up_trajec_generated(false),
    lfoot_swing_down_trajec_generated(false),
    rfoot_swing_up_trajec_generated(false),
    rfoot_swing_down_trajec_generated(false)
{
    pre_phase_index_=-1;
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

    ini_com_pos_ = bc_->getActualCOMPosition();
    ini_lf_pose_ = bc_->getActualFootPose(+Side::LEFT);
    ini_rf_pose_ = bc_->getActualFootPose(+Side::RIGHT);
    ROS_INFO_STREAM( "ini_com_pos_:" << ini_com_pos_);
    ROS_INFO_STREAM( "ini_lf_pose_:" << ini_lf_pose_.translation());
    ROS_INFO_STREAM( "ini_rf_pose_:" << ini_rf_pose_.translation());

    lfoot_swing_up_interpolator_.reset(new PoseReferenceMinJerkTopic(
                                        nh,
                                        bc_->getControllerDt(),
                                        "lfoot_swing_up_interpolator",
                                        "odom",
                                        "odom",
                                        ini_lf_pose_));

    eMatrixHom lfoot_swing_down_ini_pose = ini_lf_pose_;
    lfoot_swing_down_ini_pose.translation().z() = swing_height_;
    lfoot_swing_down_interpolator_.reset(new PoseReferenceMinJerkTopic(
                                        nh,
                                        bc_->getControllerDt(),
                                        "lfoot_swing_down_interpolator",
                                        "odom",
                                        "odom",
                                        lfoot_swing_down_ini_pose));

    rfoot_swing_up_interpolator_.reset(new PoseReferenceMinJerkTopic(
                                        nh,
                                        bc_->getControllerDt(),
                                        "rfoot_swing_up_interpolator",
                                        "odom",
                                        "odom",
                                        ini_rf_pose_));

    eMatrixHom rfoot_swing_down_ini_pose = ini_rf_pose_;
    rfoot_swing_down_ini_pose.translation().z() = swing_height_;
    rfoot_swing_down_interpolator_.reset(new PoseReferenceMinJerkTopic(
                                        nh,
                                        bc_->getControllerDt(),
                                        "rfoot_swing_down_interpolator",
                                        "odom",
                                        "odom",
                                        rfoot_swing_down_ini_pose));

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
    ROS_INFO_STREAM("support_durations_:"<<support_durations_.transpose());

    std::vector<double> support_end_times;
    key = "/support_end_times";
    if (nh.getParam(key, support_end_times)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    support_end_times_ = std2eigen(support_end_times);
    ROS_INFO_STREAM("support_end_times_:"<<support_end_times_.transpose());

    std::vector<double> support_indexes;
    key = "/support_indexes";
    if (nh.getParam(key, support_indexes)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    support_indexes_ = std2eigen(support_indexes);
    ROS_INFO_STREAM("support_indexes_:"<<support_indexes_.transpose());

    num_of_phases_ = support_indexes_.size();
    ROS_INFO_STREAM("num_of_phases_ " << num_of_phases_);

}


bool CSVWALKINGActionPrev::enterHook(const ros::Time &time)
{
    ROS_INFO_STREAM( "CSVWALKINGActionPrev::enterHook()");

//  bc_->setHybridControlFactor("leg_left_1_joint", 1.);
//  bc_->setHybridControlFactor("leg_left_2_joint", 1.);
//  bc_->setHybridControlFactor("leg_left_3_joint", 1.);
//  bc_->setHybridControlFactor("leg_left_4_joint", 1.);
//  bc_->setHybridControlFactor("leg_left_5_joint", 1.);
//  bc_->setHybridControlFactor("leg_left_6_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_1_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_2_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_3_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_4_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_5_joint", 1.);
//  bc_->setHybridControlFactor("leg_right_6_joint", 1.);


    ini_com_pos_ = bc_->getActualCOMPosition();
    ini_lf_pose_ = bc_->getActualFootPose(+Side::LEFT);
    ini_rf_pose_ = bc_->getActualFootPose(+Side::RIGHT);
    ROS_INFO_STREAM( "ini_com_pos_:" << ini_com_pos_.transpose());
    ROS_INFO_STREAM( "ini_lf_pose_:" << ini_lf_pose_.translation().transpose());
    ROS_INFO_STREAM( "ini_rf_pose_:" << ini_rf_pose_.translation().transpose());


    dt_ = bc_->getControllerDt();
    ROS_INFO_STREAM( "dt_:" << dt_.toSec());

    swing_height_ = 0.05;
    ROS_INFO_STREAM( "swing_height_:" << swing_height_);


  return true;
}



bool CSVWALKINGActionPrev::cycleHook(const ros::Time &time)
{
//    ROS_INFO_STREAM( "CSVWALKINGActionPrev::cycleHook()");


  int count = int(internal_time_.toSec()/dt_.toSec());
  if (count > com_traj_.time.size()-1) {
      count = com_traj_.time.size()-1;
  }
//    ROS_INFO_STREAM( "internal_time_.toSec():" << internal_time_.toSec());
//    ROS_INFO_STREAM( "count:" << count);

    int cur_phase_index = 0;
    for (int i=0; i < num_of_phases_; i++){
        if (internal_time_.toSec()>support_end_times_(i)){
            cur_phase_index = i+1;
            }
    }
    if (cur_phase_index > num_of_phases_-1){
        cur_phase_index = num_of_phases_-1;
    }

//    ROS_INFO_STREAM( "cur_phase_index:" << cur_phase_index);


    int cur_support_index = support_indexes_(cur_phase_index);
//    ROS_INFO_STREAM( "cur_support_index:"<< cur_support_index);

    double cur_phase_duration = support_durations_(cur_phase_index);
//    ROS_INFO_STREAM( "cur_phase_duration:"<< cur_phase_duration);

    double cur_phase_time =  cur_phase_duration - (support_end_times_(cur_phase_index)-internal_time_.toSec());
//    ROS_INFO_STREAM( "cur_phase_time:" << cur_phase_time);



    if (cur_phase_index != pre_phase_index_)
    {
        ROS_INFO_STREAM("phase switching!");
        ROS_INFO_STREAM( "internal_time_.toSec():" << internal_time_.toSec());
        ROS_INFO_STREAM( "count:" << count);
    }
    pre_phase_index_ = cur_phase_index;

    eVector3 targetCOM_pos, targetCOM_vel, targetCOM_acc;
    eVector2 global_target_cop;
    targetCOM_pos = ini_com_pos_ + com_traj_.pos.col(count) - com_traj_.pos.col(0);
    targetCOM_vel = com_traj_.vel.col(count);
    targetCOM_acc = com_traj_.acc.col(count);





    if (cur_support_index == 0) // double support
    {

        double w = sqrt(bc_->getParameters()->gravity_ / bc_->getParameters()->z_height_);
        eVector2 global_target_dcm = targetCOM_pos.head(2) + targetCOM_vel.head(2) / w;
        global_target_cop = global_target_dcm;

        control(bc_,
                rate_limiter_,
                targetCOM_pos,
                targetCOM_vel,
                global_target_cop,
                parameters_.use_rate_limited_dcm_,
                targetCOP_rate_limited_unclamped_,
                targetCOP_unclamped_);

        bc_->setWeightDistribution(0.5);
        bc_->setActualSupportType(SupporType::DS);
        bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
        bc_->setSwingLegIDs({});

    }
    else if (cur_support_index == 1) // left support
    {
        global_target_cop = ini_lf_pose_.translation().head(2);
        control(bc_,
                rate_limiter_,
                targetCOM_pos,
                targetCOM_vel,
                global_target_cop,
                parameters_.use_rate_limited_dcm_,
                targetCOP_rate_limited_unclamped_,
                targetCOP_unclamped_);

        bc_->setWeightDistribution(1.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::LEFT});
        bc_->setSwingLegIDs({Side::RIGHT});

        if (!rfoot_swing_up_trajec_generated){
            eMatrixHom target_foot_pose = ini_rf_pose_;
            target_foot_pose.translation().z() = swing_height_;
            rfoot_swing_up_interpolator_->setPoseTarget(target_foot_pose, ros::Duration(cur_phase_duration/2.0));
            rfoot_swing_down_interpolator_->setPoseTarget(ini_rf_pose_, ros::Duration(cur_phase_duration/2.0));
//            rfoot_swing_trajectory_ = SwingTrajectory3D(ini_rf_pose_, ini_rf_pose_, cur_phase_duration, swing_height_);
            rfoot_swing_up_trajec_generated = true;
            ROS_INFO_STREAM("rfoot_swing_up_trajec_generated!");
            ROS_INFO_STREAM("rfoot_swing_down_trajec_generated!");
            ROS_INFO_STREAM( "internal_time_:" << internal_time_.toSec());
            ROS_INFO_STREAM( "count:" << count);
        }


        if (cur_phase_time<cur_phase_duration/2.0){/// swing up
            rfoot_swing_up_interpolator_->integrate(internal_time_, dt_);
            bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                                     rfoot_swing_up_interpolator_->getDesiredPose(),
                                     rfoot_swing_up_interpolator_->getDesiredVelocity().first,
                                     rfoot_swing_up_interpolator_->getDesiredAcceleration().first,
                                     rfoot_swing_up_interpolator_->getDesiredVelocity().second,
                                     rfoot_swing_up_interpolator_->getDesiredAcceleration().second);
        }
        else{/// swing down
            rfoot_swing_down_interpolator_->integrate(internal_time_, dt_);
            bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                                     rfoot_swing_down_interpolator_->getDesiredPose(),
                                     rfoot_swing_down_interpolator_->getDesiredVelocity().first,
                                     rfoot_swing_down_interpolator_->getDesiredAcceleration().first,
                                     rfoot_swing_down_interpolator_->getDesiredVelocity().second,
                                     rfoot_swing_down_interpolator_->getDesiredAcceleration().second);
        }

//        // use rfoot_swing_trajectory_
//        bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
//                                 rfoot_swing_trajectory_.pose(cur_phase_time),
//                                 rfoot_swing_trajectory_.vel(cur_phase_time),
//                                 rfoot_swing_trajectory_.acc(cur_phase_time),
//                                 eVector3(0,0,0),
//                                 eVector3(0,0,0));


    }
    else if (cur_support_index == -1) // right support
    {

        global_target_cop = ini_rf_pose_.translation().head(2);
        control(bc_,
                rate_limiter_,
                targetCOM_pos,
                targetCOM_vel,
                global_target_cop,
                parameters_.use_rate_limited_dcm_,
                targetCOP_rate_limited_unclamped_,
                targetCOP_unclamped_);

        bc_->setWeightDistribution(0.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::RIGHT});
        bc_->setSwingLegIDs({Side::LEFT});


        if (!lfoot_swing_up_trajec_generated){
            eMatrixHom target_foot_pose = ini_lf_pose_;
            target_foot_pose.translation().z() = swing_height_;
            lfoot_swing_up_interpolator_->setPoseTarget(target_foot_pose, ros::Duration(cur_phase_duration/2.0));
            lfoot_swing_down_interpolator_->setPoseTarget(ini_lf_pose_, ros::Duration(cur_phase_duration/2.0));
//            lfoot_swing_trajectory_ = SwingTrajectory3D(ini_lf_pose_, ini_lf_pose_, cur_phase_duration, swing_height_);
            lfoot_swing_up_trajec_generated = true;
            ROS_INFO_STREAM("lfoot_swing_up_trajec_generated!");
            ROS_INFO_STREAM("lfoot_swing_down_trajec_generated!");
            ROS_INFO_STREAM( "internal_time_:" << internal_time_.toSec());
            ROS_INFO_STREAM( "count:" << count);
        }



        if (cur_phase_time<cur_phase_duration/2.0){/// swing up
            lfoot_swing_up_interpolator_->integrate(internal_time_, dt_);
            bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                                     lfoot_swing_up_interpolator_->getDesiredPose(),
                                     lfoot_swing_up_interpolator_->getDesiredVelocity().first,
                                     lfoot_swing_up_interpolator_->getDesiredAcceleration().first,
                                     lfoot_swing_up_interpolator_->getDesiredVelocity().second,
                                     lfoot_swing_up_interpolator_->getDesiredAcceleration().second);
        }
        else{/// swing down
            lfoot_swing_down_interpolator_->integrate(internal_time_, dt_);
            bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                                     lfoot_swing_down_interpolator_->getDesiredPose(),
                                     lfoot_swing_down_interpolator_->getDesiredVelocity().first,
                                     lfoot_swing_down_interpolator_->getDesiredAcceleration().first,
                                     lfoot_swing_down_interpolator_->getDesiredVelocity().second,
                                     lfoot_swing_down_interpolator_->getDesiredAcceleration().second);
        }


//        // use lfoot_swing_trajectory_
//        bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
//                                 lfoot_swing_trajectory_.pose(cur_phase_time),
//                                 lfoot_swing_trajectory_.vel(cur_phase_time),
//                                 lfoot_swing_trajectory_.acc(cur_phase_time),
//                                 eVector3(0,0,0),
//                                 eVector3(0,0,0));

    }






//  control(bc_,
//          rate_limiter_,
//          targetCOM_pos,
//          targetCOM_vel,
//          global_target_cop,
//          parameters_.use_rate_limited_dcm_,
//          targetCOP_rate_limited_unclamped_,
//          targetCOP_unclamped_);


//    bc_->setDesiredCOMPosition(targetCOM_pos);
//    bc_->setDesiredCOMVelocity(targetCOM_vel);
//    bc_->setDesiredCOMAcceleration(targetCOM_acc);

//    bc_->setDesiredICP(eVector3(targetCOM_pos.x(), targetCOM_pos.y(), 0.));

//
//
//    eVector2 targetCoM_pos2D(targetCOM_pos.x(), targetCOM_pos.y());
//    eVector2 targetCoM_vel2D(targetCOM_vel.x(), targetCOM_vel.y());
//    double w = sqrt(bc_->getParameters()->gravity_ / bc_->getParameters()->z_height_);
//    Eigen::Vector2d actualDCM = targetCoM_pos2D + targetCoM_vel2D / w;
//    bc_->setActualICP(eVector3(actualDCM.x(), actualDCM.y(), 0.));


  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));


  // update internal time
  internal_time_ += dt_;

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
