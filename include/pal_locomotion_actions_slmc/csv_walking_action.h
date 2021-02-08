
#ifndef _CSV_WALKING_ACTION_
#define _CSV_WALKING_ACTION_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>
#include <pal_locomotion_actions_slmc/csv_utils.h>
#include <pal_locomotion/polynomial.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk_topic.h>

namespace pal_locomotion
{

struct CSVWALKINGActionParameters
{
  CSVWALKINGActionParameters()
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

struct TrajectoryFromCSV {
    TrajectoryFromCSV(){};
    ~TrajectoryFromCSV(){};

    public:
      Eigen::VectorXd time;
      Eigen::MatrixXd pos;
      Eigen::MatrixXd vel;
};

struct ContactSequenceFromCSV {
    ContactSequenceFromCSV(){};
    ~ContactSequenceFromCSV(){};

    public:
      Eigen::VectorXd end_time;
      Eigen::MatrixXd oMi_R;
      Eigen::MatrixXd oMi_L;
      Eigen::MatrixXd oMf;
      Eigen::VectorXi type;
};

class CSVWALKINGAction : public WalkingActionBase
{
public:
  CSVWALKINGAction();

  virtual ~CSVWALKINGAction();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

  void getCSVTrajectory(const ros::NodeHandle &nh, const std::string & name, TrajectoryFromCSV & traj);
  void getCSVContactSequence(const ros::NodeHandle &nh, ContactSequenceFromCSV & cs);

  Eigen::VectorXd std2eigen(const std::vector<double> std_vec);

private:
  bool configure_interpolator_;
  bool initial_interpolation_;

  BController *bc_;
  SupporType support_type_;

  ros::Time internal_time_;
  ros::Time initial_time_;
  ros::Time control_time_;

  int cnt_;


  CSVWALKINGActionParameters parameters_;
  eVector3 actual_com_;
  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  // trajectory container
  std::vector<double> trajectory_t_;
  std::vector<int> contact_type_;
  std::vector<double> trajectory_pos_x_;
  std::vector<double> trajectory_pos_y_;
  std::vector<double> trajectory_pos_z_;
  std::vector<double> trajectory_vel_x_;
  std::vector<double> trajectory_vel_y_;
  std::vector<double> trajectory_vel_z_;

  TrajectoryFromCSV com_traj_;
  TrajectoryFromCSV lfoot_traj_;
  TrajectoryFromCSV rfoot_traj_;
  ContactSequenceFromCSV cs_;

  int current_cs_;
  bool cs_change_;
  pal_robot_tools::MinJerkGeneratorPtr force_distribution_interpolator_;
  pal_robot_tools::MinJerkGenerator l_swing_traj1_, l_swing_traj2_, r_swing_traj1_, r_swing_traj2_;

  eMatrixHom initial_left_foot_pose_, initial_right_foot_pose_;
};
}

#endif
