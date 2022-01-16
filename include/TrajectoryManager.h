#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "ros/ros.h"
#include <time.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

enum StateEnum
{
    IDLE,
    HAS_TARGET,
    WAIT,
    SHOOT,
    ABORT
};

class TrajectoryManager
{
    public:
        TrajectoryManager( ros::NodeHandle nh );
        void run();
        void load_params();

    private:
        // ROS data
        ros::NodeHandle nh_;
        ros::Subscriber trajectory_pose_sub_;
        ros::Subscriber vesc_ready_sub_;
        ros::Subscriber yaw_ready_sub_;
        ros::Subscriber abort_sub_;
        ros::Publisher vesc_cmd_pub_;
        ros::Publisher yaw_cmd_pub_;
        ros::Publisher trigger_pub_;
        ros::Publisher shot_pub_;
        ros::Publisher state_pub_;
        ros::Publisher target_pub_;
        ros::Publisher trajectory_pub_;

        // Msg data
        geometry_msgs::PoseStamped target_pose_; 
        bool vesc_ready_;
        bool yaw_ready_;
        bool abort_;

        // State data
        bool has_target_;
        StateEnum state_; 
        float target_yaw_;
        float target_velocity_;
        float contact_time_;
        ros::Time trigger_time_; 
        ros::Duration cooldown_time_;
        ros::Time cmd_sent_time_;
        ros::Duration cmd_timeout_;

        // Visualization data
        bool plot_target_;
        bool plot_traj_;

        // TF data
        std::string world_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Callbacks
        void trajectoryPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void vescReadyCallback(const std_msgs::Bool::ConstPtr& msg);
        void yawReadyCallback(const std_msgs::Bool::ConstPtr& msg);
        void abortCallback(const std_msgs::Empty::ConstPtr& msg);
        bool handleAbortSignal();
        bool handleNewCommand();
        
        // Calculation utilities
        float calculateYawAngle( const geometry_msgs::PoseStamped::ConstPtr& target_pose);
        float calculateInitialVelocity( const geometry_msgs::PoseStamped::ConstPtr& target_pose);
        float calculateInitialVelocityDrag( const geometry_msgs::PoseStamped::ConstPtr& target_pose);

        // Visualization tools
        visualization_msgs::Marker buildTargetMarker();
        visualization_msgs::Marker buildTrajectoryMarker();
        visualization_msgs::Marker buildTrajectoryMarkerDrag();

        // Constants and launcher parameters
        double launch_angle_deg_;
        double max_yaw_angle_;
        double min_yaw_angle_;
        double max_initial_velocity_;
        double const G = 9.81;
        bool use_drag_;
};

#endif
