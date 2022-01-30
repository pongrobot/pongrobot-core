#include "ros/ros.h"
#include "TrajectoryManager.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh;

    // Setup loop rate
    double loop_frq;
    if ( !nh.getParam("/rate/trajectory", loop_frq) )
    {
        ROS_ERROR("TrajectoryManager could not load param: /rate/trajectory" );
    }
    ros::Rate loop_rate(loop_frq);

    TrajectoryManager trajectoryManager(nh);

    while (ros::ok())
    {   
        ROS_WARN("TrajectoryManager RUNNNING" );
        trajectoryManager.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
