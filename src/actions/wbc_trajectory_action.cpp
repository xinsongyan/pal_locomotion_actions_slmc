#include <pal_locomotion/solvers/dcm_planner/state_machine/WBCTrajectoryAction.h>
#include <math_utils/trajectory/pose_interpolator_catmulrom.h>
#include <math_utils/math_utils.h>

namespace experimental{

WBCTrajectoryAction::WBCTrajectoryAction(BController* bController){

  id_ = 10;

  ROS_DEBUG_STREAM("Created trajectory action");

  bc_ = bController;

  internal_trajectory_time_ = ros::Time(0);

}

bool WBCTrajectoryAction::configure(BController* bController, const property_bag::PropertyBag &parameters){

  bc_ = bController;

  internal_trajectory_time_ = ros::Time(0);
  return true;
}

bool WBCTrajectoryAction::enterHook(const ros::Time &time){

  WBCInterpolator *wbc_interpolator = interpolator_buffer_.readFromRT();

  eMatrixHom actual_base_pose = createMatrix(bc_->getActualBaseOrientation(), bc_->getActualBasePosition());

  wbc_interpolator->init(internal_trajectory_time_,
                          actual_base_pose,
                          bc_->getActualTorsoOrientation(),
                          bc_->getActualUpperBodyJointPositions());


  return true;
}

/// This function is called every cycle of the loop, until the end of the Action()
bool WBCTrajectoryAction::cycleHook(const ros::Time &time){

  WBCInterpolator *wbc_interpolator = interpolator_buffer_.readFromRT();

  eMatrixHom desired_pelivis_pose;
  Eigen::Vector3d desired_pelvis_linear_vel;
  Eigen::Vector3d desired_pelvis_angular_vel;
  Eigen::Vector3d desired_pelvis_linear_acc;
  Eigen::Vector3d desired_pelvis_angular_acc;

  eQuaternion desired_torso_orientation;
  Eigen::Vector3d desired_torso_angular_vel;
  Eigen::Vector3d desired_torso_angular_acc;

  Eigen::VectorXd desired_upper_body_joint_positions(upper_joint_names_.size());
  Eigen::VectorXd desired_upper_body_joint_velocities(upper_joint_names_.size());
  Eigen::VectorXd desired_upper_body_joint_accelerations(upper_joint_names_.size());

  wbc_interpolator->interpolate(internal_trajectory_time_,
                                 desired_pelivis_pose,
                                 desired_pelvis_linear_vel,
                                 desired_pelvis_angular_vel,
                                 desired_pelvis_linear_acc,
                                 desired_pelvis_angular_acc,
                                 desired_torso_orientation,
                                 desired_torso_angular_vel,
                                 desired_torso_angular_acc,
                                 desired_upper_body_joint_positions,
                                 desired_upper_body_joint_velocities,
                                 desired_upper_body_joint_accelerations);

  bc_->setDesiredBaseOrientation(eQuaternion(desired_pelivis_pose.rotation()));
  bc_->setDesiredBaseAngularVelocity(desired_pelvis_angular_vel);
  bc_->setDesiredBaseAngularAcceleration(desired_pelvis_angular_acc);

  bc_->setDesiredTorsoOrientation(desired_torso_orientation);
  bc_->setDesiredTorsoAngularVelocity(desired_torso_angular_vel);
  bc_->setDesiredTorsoAngularAcceleration(desired_torso_angular_acc);

  bc_->setDesiredUpperBodyJointPositions(desired_upper_body_joint_positions);
  bc_->setDesiredUpperBodyJointVelocitys(desired_upper_body_joint_velocities);
  bc_->setDesiredUpperBodyJointAccelerations(desired_upper_body_joint_accelerations);

  double ICPControllerGain = bc_->getBipedParameters()->ICPControllerGain_;

  eMatrixHom actualLeftFootPose = bc_->getActualFootPose(Side::LEFT);
  eMatrixHom actualRightFootPose = bc_->getActualFootPose(Side::RIGHT);
  eMatrixHom localCoordinateFrame = interpolateBetweenTransforms(actualLeftFootPose, actualRightFootPose);
  double desiredYaw = extractYaw(localCoordinateFrame);

  eMatrixHom2d localCoordinateFrame2d;
  pal::convert(localCoordinateFrame, localCoordinateFrame2d);

  Eigen::Vector2d actualCOM = bc_->getActualCOMPosition2d();
  Eigen::Vector2d actualCOMd = bc_->getActualCOMVelocity2d();

  Eigen::Vector2d desiredCOM;
  Eigen::Vector2d desiredCOMd;
  Eigen::Vector2d desiredCOMdd;

  Eigen::Vector2d referenceCOP = localCoordinateFrame2d.translation();
  Eigen::Vector2d targetDCM = localCoordinateFrame2d.translation();
  Eigen::Vector2d targetCOP;
  Eigen::Vector2d actualDCM;

  // Compute control law
  double g = 9.8;
  double w = sqrt(g/bc_->getBipedParameters()->zHeight_);

  actualDCM = (actualCOM - localCoordinateFrame2d.translation()) + actualCOMd/w + localCoordinateFrame2d.translation();

  targetCOP = actualDCM  + ICPControllerGain*(actualDCM - targetDCM);
  desiredCOMdd = pow(w, 2)*(actualCOM - targetCOP);
  desiredCOMdd = desiredCOMdd;

  ros::Duration dt = bc_->getControllerDt();
  desiredCOM = actualCOM + actualCOMd*dt.toSec() + desiredCOMdd*pow(dt.toSec(), 2)*0.5;
  desiredCOMd = actualCOMd + desiredCOMdd*dt.toSec();

  bc_->setDesiredCOMPosition(eVector3(desiredCOM.x(), desiredCOM.y(),
                                      localCoordinateFrame.translation().z() + bc_->getBipedParameters()->zHeight_));
  bc_->setDesiredCOMVelocity(eVector3(desiredCOMd.x(), desiredCOMd.x(), 0.));
  bc_->setDesiredCOMAcceleration(eVector3(desiredCOMdd.x(), desiredCOMdd.y(), 0.));

  bc_->setDesiredBaseOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));
  bc_->setDesiredTorsoOrientation(eQuaternion(matrixRollPitchYaw(0., 0., desiredYaw)));

  bc_->setDesiredICP(eVector3(targetDCM.x(), targetDCM.y(), 0.));
  bc_->setDesiredReferenceCOP(eVector3(referenceCOP.x(), referenceCOP.y(), 0.));
  bc_->setDesiredComputedCOP(eVector3(targetCOP.x(), targetCOP.y(), 0.));
  bc_->setActualICP(eVector3(actualDCM.x(), actualDCM.y(), 0.)); // ICP aka DCM

  return true;
}

/// Return "true" if the Action has to be stopped. The default implementation use time to stop the action;
bool WBCTrajectoryAction::isOverHook(const ros::Time &time){

  return true;
}

/// when isOver()=true, this function is called and the action is removed from the queue.
bool WBCTrajectoryAction::endHook(const ros::Time &time){

  return true;
}

void WBCTrajectoryAction::callback(ihmc_msgs::WholeBodyTrajectoryRosMessageConstPtr &msg){

  WBCInterpolator *wbc_interpolator = interpolator_buffer_.readFromNonRT();
  WBCInterpolator new_interpolator(*wbc_interpolator);

  for(size_t i=0; i<msg->pelvis_trajectory_message.taskspace_trajectory_points.size(); ++i){

    const ihmc_msgs::SE3TrajectoryPointRosMessage &pelvis = msg->pelvis_trajectory_message.taskspace_trajectory_points[i];
    const ihmc_msgs::SO3TrajectoryPointRosMessage &torso = msg->chest_trajectory_message.taskspace_trajectory_points[i];

    for(size_t j=0; j<7; ++j){
     const ihmc_msgs::TrajectoryPoint1DRosMessage &left_arm = msg->left_arm_trajectory_message.joint_trajectory_messages[j].trajectory_points[i];
     const ihmc_msgs::TrajectoryPoint1DRosMessage &right_arm = msg->right_arm_trajectory_message.joint_trajectory_messages[j].trajectory_points[i];
    }

    //pelvis.
    //wbc_interpolator->addKnot( );
    interpolator_buffer_.writeFromNonRT(new_interpolator);

  }
}

}
