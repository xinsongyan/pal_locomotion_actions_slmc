#include <pal_base_ros_controller/controller_utils.h>
#include <pal_locomotion_actions_slmc/wbc_action.h>
#include <pal_wbc_controller/controllers/wbc_kinematic.h>
#include <pluginlib/class_list_macros.h>
#include <rbdl/Energy.h>
#include <rbdl/Kinematics.h>
#include <rbdl/addons/Utils.h>

using namespace pal_wbc;

namespace pal_locomotion
{
WBCAction::WBCAction()
  : cartesian_position_task_weight_(0.), cartesian_orientation_task_weight_(0.), data_for_thread_ready_(false)
{
}

WBCAction::~WBCAction()
{
    thread_.interrupt();
    // To wake up thread
    data_for_thread_ready_ = true;
    cv_.notify_all();
    thread_.join();
}

bool WBCAction::configure(ros::NodeHandle &nh, BController *bController,
                          const property_bag::PropertyBag &parameters)
{
  bc_ = bController;

  if (parameters.exists("cartesian_position_task_weight"))
  {
    parameters.getPropertyValue<double>("cartesian_position_task_weight",
                                        cartesian_position_task_weight_);
  }
  if (parameters.exists("cartesian_orientation_task_weight"))
  {
    parameters.getPropertyValue<double>("cartesian_orientation_task_weight",
                                        cartesian_orientation_task_weight_);
  }

  tp_.reset(new pal_robot_tools::TimeProfiler());
  tp_->registerTimer("wbc_kinematic_time");
  REGISTER_VARIABLE("/introspection_data", "wbc_kinematic_cycle_duration",
                    tp_->getLastCycleTimePtr("wbc_kinematic_time"), &registered_variables_);

  REGISTER_VARIABLE("/introspection_data", "wbc_kinematic_cycle_periodicity",
                    tp_->getPeriodicityPtr("wbc_kinematic_time"), &registered_variables_);

  ROS_INFO_STREAM("In WBC Action configure hook");

  double wbc_dt;
  parameters.getPropertyValue<double>("wbc_dt", wbc_dt, property_bag::RetrievalHandling::THROW);
  wbc_dt_ = ros::Duration(wbc_dt);

  counterFactor_ = round(wbc_dt / bc_->getControllerDt().toSec());
  internalWBCThreadCounter_ = 0;
  wbcSolutionReady_ = false;

  std::string stack_configuration;
  parameters.getPropertyValue("stack_configuration", stack_configuration,
                              property_bag::RetrievalHandling::THROW);
  std::string solver_type;
  parameters.getPropertyValue("solver_type", solver_type, property_bag::RetrievalHandling::THROW);

  std::vector<Side> stanceLegIDs = bc_->getStanceLegIDs();
  std::vector<std::string> footNames = bc_->getFootNames();
  std::vector<ContactDescription> contactForcesDescriptions;
  std::vector<ContactDescription> contactTorquesDescriptions;
  for (size_t i = 0; i < stanceLegIDs.size(); ++i)
  {
    contactForcesDescriptions.push_back(std::pair<std::string, Eigen::Vector3d>(
        footNames[static_cast<int>(stanceLegIDs[i])], Eigen::Vector3d::Zero()));
    contactTorquesDescriptions.push_back(std::pair<std::string, Eigen::Vector3d>(
        footNames[static_cast<int>(stanceLegIDs[i])], Eigen::Vector3d::Zero()));
  }

  std::vector<std::string> subtree_tips;
  pal_base_ros_controller::parseSubTreeTips(bController->getNodeHandle(), subtree_tips);

  bool debug = false;

  std::vector<pal_base_ros_controller::FTSensorDefinitionPtr> forceTorqueSensors;
  std::vector<pal_base_ros_controller::IMUSensorDefinitionPtr> imuSensors;

  ros::NodeHandle wbc_nh(bController->getNodeHandle(), "wbc");
  wbc_.reset(new WBCKinematic(
      wbc_nh, wbc_dt_, subtree_tips, contactForcesDescriptions,
      contactTorquesDescriptions, formulation_t::velocity, stack_configuration,
      bc_->getActualBasePosition(), bc_->getActualBaseOrientation(), bc_->getJointNames(),
      bc_->getActualJointState(), forceTorqueSensors, imuSensors, solver_type, debug));

  robot_model_ = wbc_->getRobotModel();

  generalized_state_.resize(robot_model_.q_size);
  generalized_state_d_.resize(robot_model_.qdot_size);

  base_position_interpolated_ = bc_->getActualBasePosition();
  base_orientation_interpolated_ = bc_->getActualBaseOrientation();
  base_linear_velocity_interpolated_.setZero();
  base_angular_velocity_interpolated_.setZero();
  joint_state_position_interpolated_ = bc_->getActualJointState();
  joint_state_velocity_interpolated_ =
      Eigen::VectorXd::Zero(bc_->getActualJointState().rows());

  ref_.base_position_ = base_position_interpolated_;
  ref_.base_orientation_ = base_orientation_interpolated_;
  ref_.base_linear_velocity_ = base_linear_velocity_interpolated_;
  ref_.base_angular_velocity_ = base_angular_velocity_interpolated_;
  ref_.joint_state_position_ = joint_state_position_interpolated_;
  ref_.joint_state_velocity_ = joint_state_velocity_interpolated_;

  thread_ = boost::thread(&WBCAction::wbcUpdate, this);

  struct sched_param params;
  params.sched_priority = 20;
  if (0 != pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &params))
  {
    ROS_ERROR("Could not adjust priority of the WBC kinematic thread.");
    return (false);
  }

  ROS_INFO_STREAM("End WBC Action configure hook");

  return true;
}

void WBCAction::wbcUpdate()
{
  std::unique_lock<std::mutex> lck(mtx_);

  while (true)
  {
    cv_.wait(lck, [&](){return true == data_for_thread_ready_;});
    if (thread_.interruption_requested())
    {
      return;
    }
    tp_->startTimer("wbc_kinematic_time");
    data_for_thread_ready_ = false;
    /// @todo hardcoded time values
    wbc_->update(ros::Time::now(), wbc_dt_);

    desiredState desired_state;
    desired_state.joint_state_position_.resize(bc_->getJointNames().size());
    desired_state.joint_state_velocity_.resize(bc_->getJointNames().size());

    wbc_->getDesiredJointPositions(desired_state.joint_state_position_);
    wbc_->getDesiredJointVelocities(desired_state.joint_state_velocity_);

    desired_state.base_position_ = wbc_->getDesiredBasePosition();
    desired_state.base_orientation_ = wbc_->getDesiredBaseOrientation();
    desired_state.base_linear_velocity_ = wbc_->getDesiredBaseLinearVelocity();
    desired_state.base_angular_velocity_ = wbc_->getDesiredBaseAngularVelocity();

    desiredStateBox_.set(desired_state);
    wbcSolutionReady_ = true;

    tp_->stopTime("wbc_kinematic_time");
  }
}

bool WBCAction::enterHook(const ros::Time &time)
{
  ROS_INFO_STREAM("In WBC Action enter hook");

  data_for_thread_ready_ = true;
  cv_.notify_all();

  return true;
}

/// This function is called every cycle of the loop, until the end of the
/// Action()
bool WBCAction::cycleHook(const ros::Time &time)
{
  bc_->getWBCParameters()->end_effector_position_weight_ = cartesian_position_task_weight_;
  bc_->getWBCParameters()->end_effector_orientation_weight_ = cartesian_orientation_task_weight_;

  eMatrixHom actual_left_foot_pose = bc_->getActualFootPose(+Side::LEFT);
  eMatrixHom actual_right_foot_pose = bc_->getActualFootPose(+Side::RIGHT);

  eMatrixHom local_coordinate_frame =
      interpolateBetweenTransforms(actual_left_foot_pose, actual_right_foot_pose);

  double dt = bc_->getControllerDt().toSec();

  if ((internalWBCThreadCounter_ >= counterFactor_) && wbcSolutionReady_)
  {
    base_position_interpolated_ = ref_.base_position_;
    base_orientation_interpolated_ = ref_.base_orientation_;
    joint_state_position_interpolated_ = ref_.joint_state_position_;

    desiredStateBox_.get(ref_);

    base_linear_velocity_interpolated_ = ref_.base_linear_velocity_;
    base_angular_velocity_interpolated_ = ref_.base_angular_velocity_;
    joint_state_velocity_interpolated_ = ref_.joint_state_velocity_;

    wbcSolutionReady_ = false;
    internalWBCThreadCounter_ = 0;
    data_for_thread_ready_ = true;
    cv_.notify_all();
  }
  else if ((internalWBCThreadCounter_ >= counterFactor_) && !wbcSolutionReady_)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Missed wbc update, counter: " << internalWBCThreadCounter_);
    ++internalWBCThreadCounter_;
  }
  else
  {
    ++internalWBCThreadCounter_;
  }

  base_position_interpolated_ += base_linear_velocity_interpolated_ * dt;
  eQuaternion base_orientation_interpolation =
      base_orientation_interpolated_ *
      Sophus::SO3d::exp(base_angular_velocity_interpolated_ * (internalWBCThreadCounter_ * dt))
          .unit_quaternion();

  joint_state_position_interpolated_ +=
      joint_state_velocity_interpolated_ *
      ((internalWBCThreadCounter_)*bc_->getControllerDt().toSec());

  createGeneralizedVector(base_position_interpolated_, base_orientation_interpolation,
                          joint_state_position_interpolated_, generalized_state_);

  createGeneralizedVelocityVector(base_linear_velocity_interpolated_,
                                  base_angular_velocity_interpolated_,
                                  joint_state_velocity_interpolated_, generalized_state_d_);

  RigidBodyDynamics::UpdateKinematicsCustom(robot_model_, &generalized_state_,
                                            &generalized_state_d_, NULL);

  unsigned int base_id = robot_model_.GetBodyId("base_link");
  unsigned int torso_id = robot_model_.GetBodyId("torso_2_link");
  Eigen::Vector3d desiredCOM_wbc =
      RigidBodyDynamics::CalcCOM(robot_model_, generalized_state_, false);
  Eigen::Quaterniond desired_base_orientation_global_wbc(
      RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, generalized_state_, base_id, false)
          .transpose());
  Eigen::Quaterniond desired_torso_orientation_global_wbc(
      RigidBodyDynamics::CalcBodyWorldOrientation(robot_model_, generalized_state_, torso_id, false)
          .transpose());

  /// @todo this has a bug;
  // getBodyTransform();
  eMatrixHom left_foot_pose_global_wbc = getBodyToBaseTransform(
      robot_model_, generalized_state_, bc_->getFootNames()[0], eVector3::Zero(), false);
  eMatrixHom right_foot_pose_global_wbc = getBodyToBaseTransform(
      robot_model_, generalized_state_, bc_->getFootNames()[1], eVector3::Zero(), false);
  eMatrixHom local_coordinate_frame_wbc =
      interpolateBetweenTransforms(left_foot_pose_global_wbc, right_foot_pose_global_wbc);

  Eigen::Quaterniond desired_base_orientation_local_wbc(
      local_coordinate_frame_wbc.rotation().inverse() * desired_base_orientation_global_wbc);
  Eigen::Quaterniond desired_torso_orientation_local_wbc(
      local_coordinate_frame_wbc.rotation().inverse() * desired_torso_orientation_global_wbc);

  Eigen::Vector3d desired_COM_linear_velocity_wbc = RigidBodyDynamics::CalcCOMVelocity(
      robot_model_, generalized_state_, generalized_state_d_, false);
  Eigen::Vector3d desired_base_angular_velocity_wbc = RigidBodyDynamics::CalcPointAngularVelocity(
      robot_model_, generalized_state_, generalized_state_d_, base_id, eVector3::Zero(), false);
  Eigen::Vector3d desired_torso_angular_velocity_wbc =
      RigidBodyDynamics::CalcPointAngularVelocity(robot_model_, generalized_state_,
                                                  generalized_state_d_, torso_id,
                                                  eVector3::Zero(), false);

  bc_->setDesiredBaseOrientation(
      eQuaternion(local_coordinate_frame.rotation() * desired_base_orientation_local_wbc));
  bc_->setDesiredTorsoOrientation(
      eQuaternion(local_coordinate_frame.rotation() * desired_torso_orientation_local_wbc));

  Eigen::VectorXd upper_body_joints(bc_->getUpperBodyJointNames().size());
  for (size_t i = 0; i < bc_->getUpperBodyJointNames().size(); ++i)
  {
    upper_body_joints[i] = joint_state_position_interpolated_(
        wbc_->getJointIndex(bc_->getUpperBodyJointNames()[i]));
  }
  bc_->setDesiredUpperBodyJointPositions(upper_body_joints);

  bc_->setDesiredBaseAngularVelocity(desired_base_angular_velocity_wbc);
  bc_->setDesiredTorsoAngularVelocity(desired_torso_angular_velocity_wbc);
  Eigen::VectorXd upper_body_joints_velocity(bc_->getUpperBodyJointNames().size());
  for (size_t i = 0; i < bc_->getUpperBodyJointNames().size(); ++i)
  {
    upper_body_joints_velocity[i] = joint_state_velocity_interpolated_(
        wbc_->getJointIndex(bc_->getUpperBodyJointNames()[i]));
  }
  bc_->setDesiredUpperBodyJointVelocitys(upper_body_joints_velocity);

  bc_->setDesiredBaseAngularAcceleration(eVector3::Zero());
  bc_->setDesiredTorsoAngularVelocity(eVector3::Zero());
  bc_->setDesiredTorsoAngularAcceleration(eVector3::Zero());
  bc_->setDesiredUpperBodyJointAccelerations(Eigen::VectorXd::Zero(upper_body_joints.rows()));

  // End effectors
  std::vector<std::string> ee_names = bc_->getEndEffectorNames();
  for (size_t i = 0; i < ee_names.size(); ++i)
  {
    unsigned int ee_id = robot_model_.GetBodyId(ee_names[i].c_str());

    eMatrixHom ee_pose_global_wbc = getBodyToBaseTransform(
        robot_model_, generalized_state_, ee_names[i], eVector3::Zero(), false);

    Eigen::Vector3d desired_ee_linear_velocity_wbc = RigidBodyDynamics::CalcCOMVelocity(
        robot_model_, generalized_state_, generalized_state_d_, false);
    Eigen::Vector3d desired_ee_angular_velocity_wbc = RigidBodyDynamics::CalcPointVelocity(
        robot_model_, generalized_state_, generalized_state_d_, ee_id, eVector3::Zero(), false);

    bc_->setDesiredEndEffectorPosition(ee_names[i], ee_pose_global_wbc.translation());
    bc_->setDesiredEndEffectorOrientation(ee_names[i],
                                          eQuaternion(ee_pose_global_wbc.rotation()));
    bc_->setDesiredEndEffectorLinearVelocity(ee_names[i], desired_ee_linear_velocity_wbc);
    bc_->setDesiredEndEffectorAngularVelocity(ee_names[i], desired_ee_angular_velocity_wbc);
    bc_->setDesiredEndEffectorLinearAcceleration(ee_names[i], eVector3::Zero());
    bc_->setDesiredEndEffectorAngularAcceleration(ee_names[i], eVector3::Zero());
  }

  Eigen::Vector2d actualCOM = bc_->getActualCOMPosition2d();
  Eigen::Vector2d actualCOMd = bc_->getActualCOMVelocity2d();

  double z_height = desiredCOM_wbc.z();
  // Compute control law
  double w = sqrt(bc_->getParameters()->gravity_ / z_height);

  eMatrixHom2d local_coordinate_frame_2d;
  pal::convert(local_coordinate_frame, local_coordinate_frame_2d);
  Eigen::Vector2d targetDCM = local_coordinate_frame_2d.translation();

  eVector2 actualDCM = actualCOM + actualCOMd / w;

  eVector2 targetCOP = actualDCM + bc_->getParameters()->icp_gain_ * (actualDCM - targetDCM);
  eVector2 desiredCOMdd = pow(w, 2) * (actualCOM - targetCOP);

  eVector2 desiredCOM = actualCOM + actualCOMd * dt + desiredCOMdd * pow(dt, 2) * 0.5;
  eVector2 desiredCOMd = actualCOMd + desiredCOMdd * dt;

  bc_->setDesiredCOMPosition(eVector3(desiredCOM.x(), desiredCOM.y(), z_height));
  bc_->setDesiredCOMVelocity(
      eVector3(desiredCOMd.x(), desiredCOMd.x(), desired_COM_linear_velocity_wbc.z()));
  bc_->setDesiredCOMAcceleration(eVector3(desiredCOMdd.x(), desiredCOMdd.y(), 0.));

  // Change back to global coordinates
  bc_->setDesiredICP(eVector3(targetDCM.x(), targetDCM.y(), 0.));
  bc_->setDesiredCOPReference(eVector3(targetDCM.x(), targetDCM.y(), 0.));
  bc_->setDesiredCOPComputed(eVector3(targetCOP.x(), targetCOP.y(), 0.));
  bc_->setActualICP(eVector3(actualDCM.x(), actualDCM.y(), 0.));  // ICP aka DCM

  return true;
}

/// Return "true" if the Action has to be stopped. The default implementation
/// use time to stop the action;
bool WBCAction::isOverHook(const ros::Time &time)
{
  if (bc_->getStateMachine()->queue_size() > 1)
  {
    return true;
  }

  return false;
}

/// when isOver()=true, this function is called and the action is removed from
/// the queue.
bool WBCAction::endHook(const ros::Time &time)
{
  ROS_INFO_STREAM("In WBC Action end hook");

  bc_->getWBCParameters()->end_effector_position_weight_ = 0.0;

  return true;
}
}
