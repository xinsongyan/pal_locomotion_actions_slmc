/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _WBC_ACTION_
#define _WBC_ACTION_

#include <pal_wbc_controller/stack_of_tasks_kinematic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/com_stabilizer_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>

#include <pluginlib/class_loader.h>
#include <hqp_solvers/hqp_solver.h>

#include <pal_locomotion/state_machine/walking_action_base.h>
#include <pal_locomotion/visualization/visualization.h>
#include <math_utils/geometry_tools.h>

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <condition_variable>
#include <realtime_tools/realtime_box.h>
#include <pal_wbc_controller/controllers/wbc_kinematic.h>
#include <pal_utils/time_profiler.h>

namespace pal_locomotion
{
class WBCAction : public WalkingActionBase
{
  struct desiredState
  {
    Eigen::VectorXd joint_state_position_;
    Eigen::VectorXd joint_state_velocity_;
    Eigen::Vector3d base_position_;
    Eigen::Quaterniond base_orientation_;
    Eigen::Vector3d base_linear_velocity_;
    Eigen::Vector3d base_angular_velocity_;
  };

public:
  WBCAction();

  virtual ~WBCAction();

  //    WBCAction(BController* bController);

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time);

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time);

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time);

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time);

  void wbcUpdate();

private:
  desiredState ref_;

  BController *bc_;

  // WBC
  boost::thread thread_;

  desiredState desired_state_;

  Eigen::Vector3d base_position_interpolated_;
  Eigen::Quaterniond base_orientation_interpolated_;
  Eigen::Vector3d base_linear_velocity_interpolated_;
  Eigen::Vector3d base_angular_velocity_interpolated_;
  Eigen::VectorXd joint_state_position_interpolated_;
  Eigen::VectorXd joint_state_velocity_interpolated_;

  // This states belong to the kinematic WBC
  Eigen::VectorXd generalized_state_;
  Eigen::VectorXd generalized_state_d_;

  realtime_tools::RealtimeBox<desiredState> desiredStateBox_;

  bool wbcSolutionReady_;
  int counterFactor_;
  int internalWBCThreadCounter_;
  std::mutex mtx_;
  std::atomic<bool> data_for_thread_ready_;
  std::condition_variable cv_;

  pal_wbc::WBCKinematicPtr wbc_;

  RigidBodyDynamics::Model robot_model_;

  ros::Duration wbc_dt_;

  pal_robot_tools::TimeProfilerPtr tp_;
  pal_statistics::RegistrationsRAII registered_variables_;

  double cartesian_position_task_weight_;
  double cartesian_orientation_task_weight_;
};

typedef boost::shared_ptr<WBCAction> WBCActionPtr;
}

#endif
