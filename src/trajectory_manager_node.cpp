#include "ros/ros.h"
#include "TrajectoryManager.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    TrajectoryManager trajectoryManager(nh);

    while (ros::ok())
    {
        trajectoryManager.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
