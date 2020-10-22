
#ifndef _CSV_WALKING_ACTION_PREV_
#define _CSV_WALKING_ACTION_PREV_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>
#include <pal_locomotion_actions_slmc/csv_utils.h>


namespace pal_locomotion
{

struct CSVWALKINGActionPrevParameters
{
  CSVWALKINGActionPrevParameters()
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

struct TrajectoryFromCSV2 {
    TrajectoryFromCSV2(){};
    ~TrajectoryFromCSV2(){};

    public:
      Eigen::VectorXd time;
      Eigen::MatrixXd pos;
      Eigen::MatrixXd vel;
      Eigen::MatrixXd acc;
};

struct ContactSequenceFromCSV2 {
    ContactSequenceFromCSV2(){};
    ~ContactSequenceFromCSV2(){};

    public:
      Eigen::VectorXd time;
      Eigen::VectorXd type;
};

class CSVWALKINGActionPrev : public WalkingActionBase
{
public:
  CSVWALKINGActionPrev();

  virtual ~CSVWALKINGActionPrev();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

  bool getCSVTrajectory(const ros::NodeHandle &nh, const std::string & name, TrajectoryFromCSV2 & traj);
  bool getCSVContactSequence(const ros::NodeHandle &nh, ContactSequenceFromCSV2 & cs);

  Eigen::VectorXd std2eigen(const std::vector<double> std_vec);

private:
  bool configure_interpolator_;
  bool initial_interpolation_;

  BController *bc_;
  SupporType support_type_;

  ros::Time internal_time_;
  ros::Time control_time_;
  ros::Time ds_time_;
  ros::Time ss_time_;
  ros::Time sss_time_;
  ros::Time final_time_;

  int cnt_;


  CSVWALKINGActionPrevParameters parameters_;
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
    std::vector<double> trajectory_acc_x_;
    std::vector<double> trajectory_acc_y_;
    std::vector<double> trajectory_acc_z_;

  TrajectoryFromCSV2 com_traj_;
  TrajectoryFromCSV2 lfoot_traj_;
  TrajectoryFromCSV2 rfoot_traj_;
  ContactSequenceFromCSV2 cs_;

  int current_cs_;
  bool cs_change_;

  eMatrixHom lf_pos_, rf_pos_;
};
}

#endif
