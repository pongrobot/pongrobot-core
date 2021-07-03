#include "ros/ros.h"
#include "TrajectoryManager.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh;

    // Setup loop rate
    double loop_frq;
    nh.param<double>("/rate/trajectory", loop_frq, 20.0);
    ros::Rate loop_rate(loop_frq);

    TrajectoryManager trajectoryManager(nh);

    while (ros::ok())
    {
        trajectoryManager.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
