


#include <iostream>

using namespace std;

#include "ros/ros.h"
#include "std_srvs/Trigger.h"

bool add(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{

    ROS_INFO("Trigger request!");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("trigger", add);
    ROS_INFO("Ready to trigger.");
    ros::spin();

    return 0;
}