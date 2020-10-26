
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

struct Trajectory{
        Eigen::VectorXd time;
        Eigen::MatrixXd pos;
        Eigen::MatrixXd vel;
        Eigen::MatrixXd acc;
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

  bool getTrajectoryFromRosParam(const ros::NodeHandle &nh, const std::string & name, Trajectory & traj);

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


  ros::Time begin_time_;
  ros::Duration time_from_begin_;




  int cnt_;
  ros::Duration dt_;
  double cnt_time_;



  CSVWALKINGActionPrevParameters parameters_;
  eVector3 actual_com_;
  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  // trajectory
    Trajectory com_traj_;
    Eigen::MatrixXd support_durations_;
    Eigen::MatrixXd support_indexes_;
    Eigen::MatrixXd support_end_times_;
    int num_of_phases_;

    int current_cs_;
  bool cs_change_;

  eMatrixHom lf_pos_, rf_pos_;
    eMatrixHom lf_up_pos_, rf_up_pos_;
    eVector3 ini_com_pos_;
};
}

#endif
