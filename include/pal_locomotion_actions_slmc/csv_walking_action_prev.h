
#ifndef _CSV_WALKING_ACTION_PREV_
#define _CSV_WALKING_ACTION_PREV_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk_topic.h>
//#include <pal_locomotion_actions_slmc/csv_utils.h>

#include <pal_locomotion_actions_slmc/swing_trajectory.h>

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


  BController *bc_;

  ros::Time internal_time_;
  ros::Duration dt_;


  CSVWALKINGActionPrevParameters parameters_;

  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

  // trajectory
    Trajectory com_traj_;
    Eigen::MatrixXd support_durations_;
    Eigen::MatrixXd support_indexes_;
    Eigen::MatrixXd support_end_times_;
    int num_of_phases_;

    pal_robot_tools::PoseReferenceMinJerkTopicPtr lfoot_swing_up_interpolator_;
    pal_robot_tools::PoseReferenceMinJerkTopicPtr lfoot_swing_down_interpolator_;
    pal_robot_tools::PoseReferenceMinJerkTopicPtr rfoot_swing_up_interpolator_;
    pal_robot_tools::PoseReferenceMinJerkTopicPtr rfoot_swing_down_interpolator_;
    bool lfoot_swing_up_trajec_generated;
    bool lfoot_swing_down_trajec_generated;
    bool rfoot_swing_up_trajec_generated;
    bool rfoot_swing_down_trajec_generated;

    SwingTrajectory3D lfoot_swing_trajectory_;
    SwingTrajectory3D rfoot_swing_trajectory_;

  // robot initial state
  eVector3 ini_com_pos_;
  eMatrixHom ini_lf_pose_, ini_rf_pose_;


  double swing_height_;

  int pre_phase_index_;


};
}

#endif
