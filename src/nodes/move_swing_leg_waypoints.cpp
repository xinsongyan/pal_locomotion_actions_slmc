#include <ros/ros.h>
#include <ariles/ariles_all.h>
#include <pal_control_msgs/OperationalSpaceGoal.h>
#include <math_utils/math_utils.h>
#include <pal_ros_utils/conversions.h>

struct TaskSpaceGoal : public ariles::ConfigurableBase
{
  TaskSpaceGoal()
  {
    setDefaults();
  }

  void setDefaults()
  {
  }

#define ARILES_SECTION_ID "TaskSpaceGoal"
#define ARILES_CONSTRUCTOR TaskSpaceGoal
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(target_position)                                                         \
  ARILES_ENTRY_(target_orientation)                                                      \
  ARILES_ENTRY_(target_duration)
#include ARILES_INITIALIZE

  eVector3 target_position_;
  eVector3 target_orientation_;
  double target_duration_;
};

struct TaskSpaceGoals : ariles::ConfigurableBase
{
  TaskSpaceGoals()
  {
    setDefaults();
  }

  void setDefaults()
  {
  }

#define ARILES_SECTION_ID "TaskSpaceGoals"
#define ARILES_CONSTRUCTOR TaskSpaceGoals
#define ARILES_ENTRIES ARILES_ENTRY_(goals)
#include ARILES_INITIALIZE

  std::vector<TaskSpaceGoal> goals_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_swing_leg_goals");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<pal_control_msgs::OperationalSpaceGoal>(
      "/biped_walking_dcm_controller/swing_leg_interpolator_goal", 1);

  TaskSpaceGoals waypoints;
  waypoints.readConfig<ariles::ros>(nh, "waypoints");

  for (size_t i = 0; i < waypoints.goals_.size(); ++i)
  {
    pal_control_msgs::OperationalSpaceGoal msg;
    pal::convert(waypoints.goals_[i].target_position_, msg.pose.position);
    pal::convert(quaternionRollPitchYaw(waypoints.goals_[i].target_orientation_),
                 msg.pose.orientation);

    msg.duration = ros::Duration(waypoints.goals_[i].target_duration_);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    pub.publish(msg);
    msg.duration.sleep();
  }
}
