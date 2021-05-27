#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

geometry_msgs::PoseStamped target_pose;
bool has_target = false;
bool vesc_ready = false;
bool yaw_ready = false;

enum State
{
    IDLE,
    HAS_TARGET,
    WAIT,
    SHOOT,
    ABORT
};


void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    target_pose = *msg;
    has_target = true;
}

void vescReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{
    vesc_ready = msg->data;
}

void yawReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{
    yaw_ready = msg->data;
}

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh;
    ros::Subscriber target_pose_sub = nh.subscribe("target_pose", 1000, targetPoseCallback);
    ros::Subscriber vesc_ready_sub = nh.subscribe("vesc_ready", 1000, vescReadyCallback);
    ros::Subscriber yaw_ready_sub = nh.subscribe("yaw_ready", 1000, yawReadyCallback);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
