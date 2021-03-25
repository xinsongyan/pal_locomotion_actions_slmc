
#ifndef _WALKING_ACTION_SLMC_
#define _WALKING_ACTION_SLMC_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk_topic.h>

// For realtime publisher and subscriber
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>

#include <pal_locomotion_actions_slmc/swing_trajectory.h>

#include "std_srvs/Trigger.h"

#include <boost/bind.hpp>

namespace pal_locomotion
{



struct WALKINGActionSLMCParameters
{
  WALKINGActionSLMCParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    use_rate_limited_dcm_ = false;

    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
  }
  
  bool use_rate_limited_dcm_;
  std::string filename_;

  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};

struct Trajectory{
        Eigen::VectorXd time;
        Eigen::MatrixXd pos;
        Eigen::MatrixXd vel;
        Eigen::MatrixXd acc;
};

class WALKINGActionSLMC : public WalkingActionBase
{
public:
  WALKINGActionSLMC();

  virtual ~WALKINGActionSLMC();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

  void getTrajectoryFromRosParam(const ros::NodeHandle &nh, const std::string & name, Trajectory & traj);

  void getZmpTrajectoryFromRosParam(const ros::NodeHandle &nh);

  void getSwingHeightFromRosParam(const ros::NodeHandle &nh);

  void getComGainFromRosParam(const ros::NodeHandle &nh);

  bool triggerCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

  Eigen::VectorXd std2eigen(const std::vector<double> std_vec);

  Eigen::MatrixXd stdVector2EigenMatrixXd(std::vector<double> std_vec, int row, int col);



private:

  ros::NodeHandle *nh_;

  BController *bc_;

  ros::Time internal_time_;
  ros::Duration dt_;


  WALKINGActionSLMCParameters parameters_;

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> com_states_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> foot_poses_pub_;


  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  // trajectory
    Trajectory com_traj_;
    Eigen::VectorXd zmp_t_;
    Eigen::VectorXd zmp_x_;
    Eigen::VectorXd zmp_y_;
    Eigen::VectorXd zmp_z_;
    Eigen::MatrixXd support_durations_;
    Eigen::MatrixXd support_indexes_;
    Eigen::MatrixXd support_end_times_;
    Eigen::MatrixXd foot_placements_;
    Eigen::MatrixXd foot_orientations_;

    int num_of_phases_;


    bool lfoot_swing_trajec_generated;
    bool rfoot_swing_trajec_generated;

    SwingTrajectory3D lfoot_swing_trajectory_;
    SwingTrajectory3D rfoot_swing_trajectory_;

  // robot initial state
  eVector3 ini_com_pos_;
  eVector3 current_com_pos_;
  eVector3 targetCOM_pos, targetCOM_vel, targetCOM_acc;

  std::vector<eVector3> des_foot_pos_;
  eMatrixHom actual_lf_pose, actual_rf_pose;
  std::pair<eVector3, eVector3> actual_lf_vel, actual_rf_vel;
  eMatrixHom ini_lf_pose_, ini_rf_pose_;
  eMatrixHom ini_local_pose_;

  double swing_height_;
  int cur_support_index;
  double cur_phase_duration;
  double cur_phase_time;
  
  double n_com_states_;
  double n_foot_poses_;

  double com_fb_kp_;
  double com_fb_kd_;
  double com_ff_kp_;

  int pre_phase_index_;

  // trigger service
  ros::ServiceServer trigger_service_server_;
  bool trigger_;

  const double com_height_default_ = 0.88;
};
}

#endif
