#include <pal_locomotion_actions_slmc/csv_com_action.h>
#include <pal_locomotion_actions_slmc/icp_control_utils.h>
#include <math_utils/geometry_tools.h>

using namespace math_utils;
using namespace pal_robot_tools;


#include <ros/ros.h>

//ros::init("param_loader");


rapidcsv::Document csv_com_reader_("/home/xin/catkin_ws/src/pal_locomotion_actions_slmc/trajectory/com_trajectory.csv");
namespace pal_locomotion
{
CSVCOMAction::CSVCOMAction()
  : internal_time_(ros::Time(0)), configure_interpolator_(true), initial_interpolation_(true)
{

}

CSVCOMAction::~CSVCOMAction()
{
}

bool CSVCOMAction::configure(ros::NodeHandle &nh, BController *bController,
                                const property_bag::PropertyBag &pb)
{
  bc_ = bController;
  

  rate_limiter_.reset(new HighPassRateLimiterVector2d(
      "dcm_rate_limiter", nh, bc_->getControllerDt(), parameters_.hpl_paramters_));


  pb.getPropertyValue<std::string>("filename", parameters_.filename_,
                              property_bag::RetrievalHandling::THROW);


    std::string key;
    if (nh.searchParam("com_trajectory/t", key))
    {
        std::cout << "found "<< key << std::endl;
    }else{
        std::cout << "nothing found "<< std::endl;
    }

    std::string test_string;
    if (nh.getParam("/param/string", test_string)){
        ROS_INFO_STREAM("Successfully load /param/string from ros parameter server!");
    }else{
        ROS_INFO_STREAM("Fail to load /param/string from ros parameter server!");
    }

    std::vector<double> t;
    if (nh.getParam("com_trajectory/t", t)){
        ROS_INFO_STREAM("Successfully load com_trajectory/t from ros parameter server!");
    }else{
        ROS_INFO_STREAM("Fail to load com_trajectory/t from ros parameter server!");
    }


  return true;
}

bool CSVCOMAction::enterHook(const ros::Time &time)
{
  support_type_ = bc_->getActualSupportType();

  int csv_size = csv_com_reader_.GetRowCount();
  internal_time_ = time;
  control_time_ = internal_time_ + ros::Duration(csv_com_reader_.GetRow<float>(csv_size-1)[0]);
  actual_com_= bc_->getActualCOMPosition();
  cnt_ = 0;

  return true;
}

bool CSVCOMAction::cycleHook(const ros::Time &time)
{
 
  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame;
 

  // define local frame
  if (support_type_ == +SupporType::DS)
  {
    local_coordinate_frame =
        interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);
  }
  else
  {
    PAL_THROW_DEFAULT("not supported for Single Support");
  }


  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);

  eVector3 targetCOM, targetCOM_vel;
  targetCOM = actual_com_;
  targetCOM_vel.setZero();

  if (cnt_ < csv_com_reader_.GetRowCount()){
    for (int i=1 ; i<4; i++) {
          targetCOM(i-1) = actual_com_(i-1)+ csv_com_reader_.GetRow<float>(cnt_)[i] - csv_com_reader_.GetRow<float>(0)[i];
          targetCOM_vel(i-1) =  csv_com_reader_.GetRow<float>(cnt_)[i+3];
    }
    cnt_ += 1;
  }
  else
  {
    for (int i=1 ; i<4; i++) {
     targetCOM(i-1) = actual_com_(i-1) + csv_com_reader_.GetRow<float>(csv_com_reader_.GetRowCount()-1)[i] - csv_com_reader_.GetRow<float>(0)[i];
     targetCOM_vel(i-1) =  csv_com_reader_.GetRow<float>(csv_com_reader_.GetRowCount()-1)[i+3];
    }
  }

  if (fabs((internal_time_ - control_time_).toSec()) < 1e-3){
    ROS_INFO_STREAM("Done");
    std::cout << cnt_ <<std::endl;
  }

  eVector2 global_target_cop = targetCOM.head(2);

  control(bc_, rate_limiter_, targetCOM, targetCOM_vel, global_target_cop,
          parameters_.use_rate_limited_dcm_, targetCOP_rate_limited_unclamped_,
          targetCOP_unclamped_);

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., 0)));
  
  internal_time_ += bc_->getControllerDt();

  return true;
}

bool CSVCOMAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }
  return false;
}

bool CSVCOMAction::endHook(const ros::Time &time)
{
  return true;
}
}
