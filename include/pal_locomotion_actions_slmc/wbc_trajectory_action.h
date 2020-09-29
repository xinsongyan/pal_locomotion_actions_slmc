/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _WBC_TRAJECTORY_ACTION_
#define _WBC_TRAJECTORY_ACTION_

#include <pal_locomotion/solvers/dcm_planner/state_machine/walking_action_base.h>
#include <pal_locomotion/biped_controller.h>
#include <pal_locomotion/visualization/visualization.h>
#include <pal_robot_tools/GeometryTools.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include <realtime_tools/realtime_buffer.h>

namespace pal_locomotion
{
class WBCTrajectoryAction : public WalkingActionBase
{
public:
  class WBCInterpolator
  {
  public:
    WBCInterpolator()
    {
    }

    virtual ~WBCInterpolator()
    {
    }

    void init(const ros::Time &time, const eMatrixHom &base_pose,
              const eQuaternion &torso_orientation,
              const Eigen::VectorXd &upper_body_joint_positions)
    {
      pelvis_traj_interpolator_.clearAndSet(base_pose, time.toSec());

      torso_traj_interpolator_.addKnot(time.toSec(), torso_orientation);

      upper_body_traj_interpolator_.insertPoint(upper_body_joint_positions, time.toSec());
    }

    void interpolate(const ros::Time &time, eMatrixHom &desired_pelivis_pose,
                     Eigen::Vector3d &desired_pelvis_linear_vel,
                     Eigen::Vector3d &desired_pelvis_angular_vel,
                     Eigen::Vector3d &desired_pelvis_linear_acc,
                     Eigen::Vector3d &desired_pelvis_angular_acc,
                     eQuaternion &desired_torso_orientation,
                     Eigen::Vector3d &torso_angular_vel, Eigen::Vector3d &torso_angular_acc,
                     Eigen::VectorXd &upper_body_joint_positions,
                     Eigen::VectorXd &upper_body_joint_velocities,
                     Eigen::VectorXd &upper_body_joint_accelerations)
    {
      pelvis_traj_interpolator_.interpolate(
          time.toSec(), desired_pelivis_pose, desired_pelvis_linear_vel,
          desired_pelvis_angular_vel, desired_pelvis_linear_acc, desired_pelvis_angular_acc);

      QuaternionTrajectory::Element torso_traj_element =
          torso_traj_interpolator_.evaluate_catmul_rom(time.toSec());
      desired_torso_orientation = torso_traj_element.value;
      torso_angular_vel = torso_traj_element.vel;
      torso_angular_acc = torso_traj_element.accel;

      upper_body_traj_interpolator_.interpolate(time.toSec(), &upper_body_joint_positions,
                                                &upper_body_joint_velocities,
                                                &upper_body_joint_accelerations);
    }

  private:
    PoseInterpolatorCatmulRom pelvis_traj_interpolator_;
    QuaternionTrajectory torso_traj_interpolator_;
    TrajectoryEigenXd upper_body_traj_interpolator_;
  };

  typedef boost::shared_ptr<WBCInterpolator> WBCInterpolatorPtr;

  WBCTrajectoryAction()
  {
  }

  virtual ~WBCTrajectoryAction()
  {
  }

  WBCTrajectoryAction(BController *bController);

  bool configure(BController *bController, const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time);

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time);

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time);

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time);

private:
  void callback(ihmc_msgs::WholeBodyTrajectoryRosMessageConstPtr &msg);

  BController *bc_;
  ros::Subscriber traj_sub_;
  std::vector<std::string> upper_joint_names_;
  ros::Time internal_trajectory_time_;

  realtime_tools::RealtimeBuffer<WBCInterpolator> interpolator_buffer_;
};

typedef boost::shared_ptr<WBCTrajectoryAction> WBCTrajectoryActionPtr;
}

#endif
