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
    lfoot_swing_trajec_generated(false),
    rfoot_swing_trajec_generated(false)
{
    pre_phase_index_=-1;
}

CSVWALKINGActionPrev::~CSVWALKINGActionPrev()
{
}

bool CSVWALKINGActionPrev::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
    ROS_INFO_STREAM( "CSVWALKINGActionPrev::configure()");
  bc_ = bController;


    dt_ = bc_->getControllerDt();
    ROS_INFO_STREAM( "dt_:" << dt_.toSec());

//    swing_height_ = 0.05;
//    ROS_INFO_STREAM( "swing_height_:" << swing_height_);

    getTrajectoryFromRosParam(nh, "com", com_traj_);
//    getTrajectoryFromRosParam(nh, "lfoot", lfoot_traj_);
//    getTrajectoryFromRosParam(nh, "rfoot", rfoot_traj_);
    getZmpTrajectoryFromRosParam(nh);
    getSwingHeightFromRosParam(nh);

    rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));





    return true;
}

void CSVWALKINGActionPrev::getSwingHeightFromRosParam(const ros::NodeHandle &nh){
    std::string key;
    key = "/swing_height";
    if (nh.getParam(key, swing_height_)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
}

void CSVWALKINGActionPrev::getZmpTrajectoryFromRosParam(const ros::NodeHandle &nh){
    std::string key;
    key = "/zmp_trajectory/t";
    std::vector<double> std_zmp_t;
    if (nh.getParam(key, std_zmp_t)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    Eigen::MatrixXd eigen_zmp_t = Eigen::Map<Eigen::VectorXd>(std_zmp_t.data(), std_zmp_t.size());

    key = "/zmp_trajectory/x";
    std::vector<double> std_zmp_x;
    if (nh.getParam(key, std_zmp_x)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    Eigen::MatrixXd eigen_zmp_x = Eigen::Map<Eigen::VectorXd>(std_zmp_x.data(), std_zmp_x.size());

    key = "/zmp_trajectory/y";
    std::vector<double> std_zmp_y;
    if (nh.getParam(key, std_zmp_y)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    Eigen::MatrixXd eigen_zmp_y = Eigen::Map<Eigen::VectorXd>(std_zmp_y.data(), std_zmp_y.size());

    key = "/zmp_trajectory/z";
    std::vector<double> std_zmp_z;
    if (nh.getParam(key, std_zmp_z)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    Eigen::MatrixXd eigen_zmp_z = Eigen::Map<Eigen::VectorXd>(std_zmp_z.data(), std_zmp_z.size());

    zmp_t_ = eigen_zmp_t;
    zmp_x_ = eigen_zmp_x;
    zmp_y_ = eigen_zmp_y;
    zmp_z_ = eigen_zmp_z;
}

void CSVWALKINGActionPrev::getTrajectoryFromRosParam(const ros::NodeHandle &nh, const std::string & name, Trajectory & traj){
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


    std::vector<double> foot_placement;
    key = "/foot_placements/data";
    if (nh.getParam(key, foot_placement)){
        ROS_INFO_STREAM("Successfully load " + key);
        // ROS_INFO_STREAM("original foot placement :\n"+foot_placement);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    int row_num;
    key = "/foot_placements/row_number";
    if (nh.getParam(key, row_num)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }

    int col_num;
    key = "/foot_placements/col_number";
    if (nh.getParam(key, col_num)){
        ROS_INFO_STREAM("Successfully load " + key);
    }else{
        ROS_INFO_STREAM("Fail to load " + key);
    }
    foot_placements_ = stdVector2EigenMatrixXd(foot_placement, row_num, col_num);
    ROS_INFO_STREAM("foot placement_ :\n"<<foot_placements_);

}


bool CSVWALKINGActionPrev::enterHook(const ros::Time &time)
{
    ROS_INFO_STREAM( "CSVWALKINGActionPrev::enterHook()");

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


    ini_com_pos_ = bc_->getActualCOMPosition();
    ini_lf_pose_ = bc_->getActualFootPose(+Side::LEFT);
    ini_rf_pose_ = bc_->getActualFootPose(+Side::RIGHT);
    ini_local_pose_ = interpolateBetweenTransforms(ini_lf_pose_, ini_rf_pose_);
    ROS_INFO_STREAM( "ini_com_pos_:" << ini_com_pos_.transpose());
    ROS_INFO_STREAM( "ini_lf_pose_:" << ini_lf_pose_.translation().transpose());
    ROS_INFO_STREAM( "ini_rf_pose_:" << ini_rf_pose_.translation().transpose());
    ROS_INFO_STREAM( "ini_local_pose_:" << ini_local_pose_.translation());



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
    // ROS_INFO_STREAM( "count:" << count);

    int cur_phase_index = 0;
    for (int i=0; i < num_of_phases_; i++){
        if (internal_time_.toSec()>support_end_times_(i)){
            cur_phase_index = i+1;
            rfoot_swing_trajec_generated = false;
            lfoot_swing_trajec_generated = false;
            }
    }
    if (cur_phase_index > num_of_phases_-1){
        cur_phase_index = num_of_phases_-1;
    }

    // ROS_INFO_STREAM( "cur_phase_index:" << cur_phase_index);


    int cur_support_index = support_indexes_(cur_phase_index);
    // ROS_INFO_STREAM( "cur_support_index:"<< cur_support_index);

    double cur_phase_duration = support_durations_(cur_phase_index);
    // ROS_INFO_STREAM( "cur_phase_duration:"<< cur_phase_duration);

    double cur_phase_time =  cur_phase_duration - (support_end_times_(cur_phase_index)-internal_time_.toSec());
    // ROS_INFO_STREAM( "cur_phase_time:" << cur_phase_time);







    eVector3 targetCOM_pos, targetCOM_vel, targetCOM_acc;
    eVector2 global_target_cop;

    // targetCOM_pos = ini_com_pos_;
    // targetCOM_pos.x() = com_traj_.pos.col(count)[0];
    // targetCOM_pos.y() = com_traj_.pos.col(count)[1];
    targetCOM_pos = com_traj_.pos.col(count);
    targetCOM_vel = com_traj_.vel.col(count);
    targetCOM_acc = com_traj_.acc.col(count);

    // ROS_INFO_STREAM( "targetCOM_pos:" << targetCOM_pos.transpose());
    // ROS_INFO_STREAM( "targetCOM_vel:" << targetCOM_vel.transpose());
    // ROS_INFO_STREAM( "targetCOM_acc:" << targetCOM_acc.transpose());

    double w = sqrt(bc_->getParameters()->gravity_ / bc_->getParameters()->z_height_);
    // ROS_INFO_STREAM("w is " << w);
    eVector2 global_target_dcm = targetCOM_pos.head(2) + targetCOM_vel.head(2) / w;
    // ROS_INFO_STREAM("global target dcm is " << global_target_dcm);
    global_target_cop = global_target_dcm;
    // ROS_INFO_STREAM("global_target_cop 1 is " << global_target_cop);
    global_target_cop = eVector2(zmp_x_(count), zmp_y_(count));
    // ROS_INFO_STREAM("global_target_cop 2 is " << global_target_cop);



//    ROS_INFO_STREAM("icp_control() in!");
    icp_control(bc_,
            rate_limiter_,
            targetCOM_pos,
            targetCOM_vel,
            global_target_cop,
            parameters_.use_rate_limited_dcm_,
            targetCOP_rate_limited_unclamped_,
            targetCOP_unclamped_);
//    ROS_INFO_STREAM("icp_control() out!");


//    eMatrixHom cur_lf_pose = bc_->getActualFootPose(+Side::LEFT);
//    eMatrixHom cur_rf_pose = bc_->getActualFootPose(+Side::RIGHT);
//    eMatrixHom cur_local_pose = interpolateBetweenTransforms(cur_lf_pose, cur_rf_pose);

//    targetCOM_pos = cur_local_pose.translation();
//    targetCOM_pos(2) = cur_local_pose.translation().z() + bc_->getParameters()->z_height_;
//    targetCOM_vel = eVector3(0.0, 0.0, 0.0);

//    bc_->setDesiredCOMPosition(targetCOM_pos);
//    bc_->setDesiredCOMVelocity(targetCOM_vel);
//    bc_->setDesiredCOMAcceleration(targetCOM_acc);
//    bc_->setDesiredCOMAcceleration(10*(targetCOM_pos - bc_->getActualCOMPosition()) + 1*(targetCOM_vel - bc_->getActualCOMVelocity()) + 1*targetCOM_acc);
//    bc_->setDesiredICP(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));
//    bc_->setDesiredCOPReference(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));
//    bc_->setDesiredCOPComputed(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));




    if (cur_support_index == 0) // double support
    {

//        bc_->setDesiredCOMPosition(targetCOM_pos);
//        bc_->setDesiredCOMVelocity(targetCOM_vel);
//        bc_->setDesiredCOMAcceleration(targetCOM_acc);
//        bc_->setDesiredCOPReference(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));
//        bc_->setDesiredCOPComputed(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));

        bc_->setWeightDistribution(0.5);
        bc_->setActualSupportType(SupporType::DS);
        bc_->setStanceLegIDs({Side::LEFT, Side::RIGHT});
        bc_->setSwingLegIDs({});

        // ROS_INFO_STREAM("DOUBLE SUPPORT !!!!!!!!!!!!!!!!!!!!!");





    }
    else if (cur_support_index == 1) // left support
    {
//        global_target_cop = ini_lf_pose_.translation().head(2);
//        bc_->setDesiredCOMPosition(targetCOM_pos);
//        bc_->setDesiredCOMVelocity(targetCOM_vel);
//        bc_->setDesiredCOMAcceleration(targetCOM_acc);
//        bc_->setDesiredCOPReference(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));
//        bc_->setDesiredCOPComputed(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));


        bc_->setWeightDistribution(1.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::LEFT});
        bc_->setSwingLegIDs({Side::RIGHT});

        if (!rfoot_swing_trajec_generated){
              eMatrixHom start_rf_pose_, final_rf_pose_;
            start_rf_pose_ = ini_rf_pose_;
            final_rf_pose_ = ini_rf_pose_;
            start_rf_pose_.translation().x() = foot_placements_(cur_phase_index-1, 3); //  + ini_com_pos_.x()
            start_rf_pose_.translation().y() = foot_placements_(cur_phase_index-1, 4); //  + ini_com_pos_.y()
            start_rf_pose_.translation().z() = foot_placements_(cur_phase_index-1, 5);
            final_rf_pose_.translation().x() = foot_placements_(cur_phase_index, 3); //  + ini_com_pos_.x()
            final_rf_pose_.translation().y() = foot_placements_(cur_phase_index, 4); //  + ini_com_pos_.y()
            final_rf_pose_.translation().z() = foot_placements_(cur_phase_index, 5);
            ROS_INFO_STREAM("Foot x is " << foot_placements_(cur_phase_index, 3));
            ROS_INFO_STREAM("Foot y is " << foot_placements_(cur_phase_index, 4));
            ROS_INFO_STREAM("LEFT SUPPORT !!!!!!!!!!!!!!!!!!!!!");
            Eigen::Vector3d ini_vel(0,0,0);
            Eigen::Vector3d fin_vel(0,0,0);
            rfoot_swing_trajectory_ = SwingTrajectory3D(start_rf_pose_, final_rf_pose_, cur_phase_duration, swing_height_, ini_vel, fin_vel);
            rfoot_swing_trajec_generated = true;
            // ROS_INFO_STREAM("rfoot_swing_trajec_generated!");
            // ROS_INFO_STREAM("rfoot_swing_down_trajec_generated!");
            // ROS_INFO_STREAM( "internal_time_:" << internal_time_.toSec());
            // ROS_INFO_STREAM( "count:" << count);
        }

//         use rfoot_swing_trajectory_
        bc_->setDesiredFootState(static_cast<int>(+Side::RIGHT),
                                 rfoot_swing_trajectory_.pose(cur_phase_time),
                                 rfoot_swing_trajectory_.vel(cur_phase_time),
                                 rfoot_swing_trajectory_.acc(cur_phase_time),
                                 eVector3(0,0,0),
                                 eVector3(0,0,0));




    }
    else if (cur_support_index == -1) // right support
    {


//        global_target_cop = ini_rf_pose_.translation().head(2);
//        bc_->setDesiredCOMPosition(targetCOM_pos);
//        bc_->setDesiredCOMVelocity(targetCOM_vel);
//        bc_->setDesiredCOMAcceleration(targetCOM_acc);
//        bc_->setDesiredCOPReference(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));
//        bc_->setDesiredCOPComputed(eVector3(global_target_cop.x(), global_target_cop.y(), 0.));


        bc_->setWeightDistribution(0.0);
        bc_->setActualSupportType(SupporType::SS);
        bc_->setStanceLegIDs({Side::RIGHT});
        bc_->setSwingLegIDs({Side::LEFT});


        if (!lfoot_swing_trajec_generated){
            eMatrixHom start_lf_pose_, final_lf_pose_;
            start_lf_pose_ = ini_lf_pose_;
            final_lf_pose_ = ini_lf_pose_;
            start_lf_pose_.translation().x() = foot_placements_(cur_phase_index-1, 0); //  + ini_com_pos_.x()
            start_lf_pose_.translation().y() = foot_placements_(cur_phase_index-1, 1); //  + ini_com_pos_.y()
            start_lf_pose_.translation().z() = foot_placements_(cur_phase_index-1, 2);
            final_lf_pose_.translation().x() = foot_placements_(cur_phase_index, 0); //  + ini_com_pos_.x()
            final_lf_pose_.translation().y() = foot_placements_(cur_phase_index, 1); //  + ini_com_pos_.y()
            final_lf_pose_.translation().z() = foot_placements_(cur_phase_index, 2);
            ROS_INFO_STREAM("Foot x is " << foot_placements_(cur_phase_index, 0));
            ROS_INFO_STREAM("Foot y is " << foot_placements_(cur_phase_index, 1));
            ROS_INFO_STREAM("RIGHT SUPPORT !!!!!!!!!!!!!!!!!!!!!");
            Eigen::Vector3d ini_vel(0,0,0);
            Eigen::Vector3d fin_vel(0,0,0);
            lfoot_swing_trajectory_ = SwingTrajectory3D(start_lf_pose_, final_lf_pose_, cur_phase_duration, swing_height_, ini_vel, fin_vel);
            lfoot_swing_trajec_generated = true;
            ROS_INFO_STREAM("lfoot_swing_trajec_generated!");
            ROS_INFO_STREAM("lfoot_swing_down_trajec_generated!");
            ROS_INFO_STREAM( "internal_time_:" << internal_time_.toSec());
            ROS_INFO_STREAM( "count:" << count);
        }



        // use lfoot_swing_trajectory_
        bc_->setDesiredFootState(static_cast<int>(+Side::LEFT),
                                 lfoot_swing_trajectory_.pose(cur_phase_time),
                                 lfoot_swing_trajectory_.vel(cur_phase_time),
                                 lfoot_swing_trajectory_.acc(cur_phase_time),
                                 eVector3(0,0,0),
                                 eVector3(0,0,0));



    }






//  control(bc_,
//          rate_limiter_,
//          targetCOM_pos,
//          targetCOM_vel,
//          global_target_cop,
//          parameters_.use_rate_limited_dcm_,
//          targetCOP_rate_limited_unclamped_,
//          targetCOP_unclamped_);



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


Eigen::MatrixXd CSVWALKINGActionPrev::stdVector2EigenMatrixXd(std::vector<double> std_vec, int row, int col){
    Eigen::MatrixXd eigen_mat = Eigen::Map<Eigen::MatrixXd>(std_vec.data(), col, row).transpose();
    return eigen_mat;
}
}
