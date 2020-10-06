
#ifndef _CSV_COM_ACTION_
#define _CSV_COM_ACTION_

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>
#include <pal_locomotion_actions_slmc/csv_utils.h>


namespace pal_locomotion
{

struct CSVCOMActionParameters
{
  CSVCOMActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    use_rate_limited_dcm_ = false;

    hpl_paramters_.filter_params_->filter_cutoff_ = 5.;
    hpl_paramters_.rate_limiter_params_->max_rate_ = 0.5;
  }
  
#define ARILES_SECTION_ID "CSVCOMActionParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(use_rate_limited_dcm)                                                    \
  ARILES_ENTRY_(filename)                                                             \                                             \
#include ARILES_INITIALIZE

  bool use_rate_limited_dcm_;
  std::string filename_;

  math_utils::HighPassRateLimiterParameters hpl_paramters_;
};

class CSVCOMAction : public WalkingActionBase
{
public:
  CSVCOMAction();

  virtual ~CSVCOMAction();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time) override;

  bool cycleHook(const ros::Time &time) override;

  bool isOverHook(const ros::Time &time) override;

  bool endHook(const ros::Time &time) override;

  bool getComTrajectory(ros::NodeHandle &nh);
  Eigen::VectorXd std2eigen(const std::vector<double> std_vec);

private:
  bool configure_interpolator_;
  bool initial_interpolation_;

  BController *bc_;
  SupporType support_type_;

  ros::Time internal_time_;
  ros::Time control_time_;
  int cnt_;


  CSVCOMActionParameters parameters_;
  eVector3 actual_com_;
  math_utils::HighPassRateLimiterVector2dPtr rate_limiter_;
  eVector2 targetCOP_rate_limited_unclamped_;
  eVector2 targetCOP_unclamped_;

    // com trajectory container
    std::vector<double> com_trajectory_t_;
    std::vector<double> com_trajectory_pos_x_;
    std::vector<double> com_trajectory_pos_y_;
    std::vector<double> com_trajectory_pos_z_;
    std::vector<double> com_trajectory_vel_x_;
    std::vector<double> com_trajectory_vel_y_;
    std::vector<double> com_trajectory_vel_z_;

    Eigen::VectorXd com_t_;
    Eigen::MatrixXd com_pos_;
    Eigen::MatrixXd com_vel_;

};
}

#endif
