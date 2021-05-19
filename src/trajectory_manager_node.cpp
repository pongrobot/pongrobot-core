#include "ros/ros.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh;

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
