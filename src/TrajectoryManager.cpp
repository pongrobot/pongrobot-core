#include "TrajectoryManager.h"

TrajectoryManager::
TrajectoryManager( ros::NodeHandle nh ):
    has_target_(false),
    vesc_ready_(false),
    yaw_ready_(false),
    state_(StateEnum::IDLE),
    trigger_time_(ros::Time::now()),
    cmd_sent_time_(ros::Time::now()),
    cooldown_time_(10),
    cmd_timeout_(30),
    launch_angle_deg_(35)
{
    nh_ = nh;

    // setup subscribers
    trajectory_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("trajectory_pose", 1, &TrajectoryManager::trajectoryPoseCallback, this);
    vesc_ready_sub_ = nh_.subscribe<std_msgs::Bool>("vesc_ready", 1, &TrajectoryManager::vescReadyCallback, this);
    yaw_ready_sub_ = nh_.subscribe<std_msgs::Bool>("yaw_ready", 1, &TrajectoryManager::yawReadyCallback, this);
    abort_sub_ = nh_.subscribe<std_msgs::Empty>("abort", 1, &TrajectoryManager::abortCallback, this);

    // setup publishers
    vesc_cmd_pub_= nh.advertise<std_msgs::Float32>("velocity_cmd", 1);
    yaw_cmd_pub_= nh.advertise<std_msgs::Int8>("yaw_cmd", 1);
    trigger_pub_= nh.advertise<std_msgs::Empty>("trigger", 1);
    shot_pub_ = nh.advertise<geometry_msgs::PoseStamped>("shot",1);
    state_pub_ = nh.advertise<std_msgs::Int8>("trajectory_manager_state",1);
}

void
TrajectoryManager::
trajectoryPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Store message
    target_pose_ = *msg;

    // Make sure Z value is negative
    if ( msg->pose.position.z <= 0 )
    {
        // Calculate trajectory
        target_yaw_ = calculateYawAngle(msg);
        target_velocity_ = calculateInitialVelocity(msg);

        // Check bounds
        bool yaw_in_range = -85 < target_yaw_ && target_yaw_ < 85;
        bool v_in_range = target_velocity_ >= 0 && target_velocity_ < 10000;

        if ( yaw_in_range && v_in_range )
        {
            has_target_ = true;
            abort_ = false; // overwrite abort signal if present
        }
        else
        {
            has_target_ = true;

            if (!yaw_in_range)
            {
                ROS_WARN("Calculated Yaw: %f deg is out of range", target_yaw_);            
            }

            if (!v_in_range)
            {
                ROS_WARN("Calculated Initial Velocity: %f m/s is out of range", target_velocity_);            
            }
        }
    }

}

void
TrajectoryManager::
vescReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{
    vesc_ready_ = msg->data;
}

void
TrajectoryManager::
yawReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{
    yaw_ready_ = msg->data;
}

void
TrajectoryManager::
abortCallback(const std_msgs::Empty::ConstPtr& msg)
{
    abort_ = true;
    has_target_ = false; // Overwrite new command is there is one
}

float
TrajectoryManager::
calculateYawAngle(const geometry_msgs::PoseStamped::ConstPtr& target_pose )
{
    // Calculate the yaw angle needed to hit the cup at a given pose
    double yaw = atan(target_pose->pose.position.y/target_pose->pose.position.x) * 180/M_PI;

    // TODO: Connect calculation
    return -40.f;
}

float
TrajectoryManager::
calculateInitialVelocity(const geometry_msgs::PoseStamped::ConstPtr& target_pose )
{
    // Calculate the initial velocity of the ball neglecting drag
    double d = sqrt( pow(target_pose->pose.position.x, 2) + pow(target_pose->pose.position.y, 2) );
    double z_c = target_pose->pose.position.z;
    double theta = launch_angle_deg_ * (M_PI / 180.f);
    
    // Calculate time the ball hits the cup
    double contact_time = sqrt( ( 2.f * d * tan(theta) - z_c ) / G );
    double launch_v = d / ( cos(theta) * contact_time );

    // TODO: Connect calcualtion
    return 30.f;
}

bool
TrajectoryManager::
handleAbortSignal()
{
    bool status = false;    

    // Check for abort signal
    if ( abort_ )
    {
        // Transition to ABORT
        abort_ = false;
        status = true;
        state_ = StateEnum::ABORT;
    }

    return status;
}

bool
TrajectoryManager::
handleNewCommand()
{
    bool status = false;    

    // Check for abort signal
    if ( has_target_ )
    {
        // Transition to HAS_TARGET
        has_target_ = false;
        status = true;
        state_ = StateEnum::HAS_TARGET;
    }

    return status;
}

void
TrajectoryManager::
run()
{
    // Publish launcher state for debugging
    std_msgs::Int8 state_int;
    state_int.data = state_;
    state_pub_.publish(state_int);

    // Main state machine
    switch(state_)
    {
        case StateEnum::IDLE:
        {
            // Check for new command
            if( handleNewCommand() )
            {
                ROS_INFO("State Transition: IDLE->HAS_TARGET");
            }
            break;
        }
        case StateEnum::HAS_TARGET:
        {
            // Check for abort signal
            if ( handleAbortSignal() )
            {
                ROS_INFO("State Transition: HAS_TARGET->ABORT");
                break;
            }

            // Check for new command
            if ( handleNewCommand() )
            {
                ROS_INFO("State Transition: HAS_TARGET->HAS_TARGET");
                break;
            }

            // Send yaw command
            std_msgs::Int8 yaw_cmd;
            yaw_cmd.data = (int) target_yaw_;
            yaw_cmd_pub_.publish(yaw_cmd);

            // Send initial velocity command
            std_msgs::Float32 velocity_cmd;
            velocity_cmd.data = target_velocity_;
            vesc_cmd_pub_.publish(velocity_cmd);

            // Transition to WAIT(commands sent to subsystems)
            cmd_sent_time_ = ros::Time::now();
            state_ = StateEnum::WAIT;
            ROS_INFO("State Transition: HAS_TARGET->WAIT");

            break;
        }
        case StateEnum::WAIT:
        {
            // Check for abort signal
            if ( handleAbortSignal() )
            {
                ROS_INFO("State Transition: WAIT->ABORT");
                break;
            }

            // Check for new command
            if ( handleNewCommand() )
            {
                ROS_INFO("State Transition: WAIT->HAS_TARGET");
                break;
            }

            // Check if subsystems have timed out
            if ( ros::Time::now() - cmd_sent_time_ > cmd_timeout_ )
            {
                // Transition to HAS_TARGET
                state_ = StateEnum::HAS_TARGET;
                ROS_INFO("State Transition: WAIT->HAS_TARGET");
            }

            // Check if all subsystems are ready
            if ( vesc_ready_ && yaw_ready_ )
            {
                trigger_pub_.publish(std_msgs::Empty());
                
                // Transition to SHOOT
                trigger_time_ = ros::Time::now();
                state_ = StateEnum::SHOOT;
                ROS_INFO("State Transition: WAIT->SHOOT");
            }

            break;
        }
        case StateEnum::SHOOT:
        {
            // Wait for launcher to cool down
            if ( (ros::Time::now() - trigger_time_) > cooldown_time_ )
            {
                // Transition to IDLE
                state_ = StateEnum::IDLE;
                ROS_INFO("State Transition: SHOOT->IDLE");
            }

            break;
        }
        case StateEnum::ABORT:
        {
            // Send zero velocity command to the VESC
            std_msgs::Float32 velocity_cmd;
            velocity_cmd.data = 0.f;
            vesc_cmd_pub_.publish(velocity_cmd);

            // Publish shot confirmation
            shot_pub_.publish(target_pose_);

            // Transition to IDLE
            state_ = StateEnum::IDLE;
            ROS_INFO("State Transition: ABORT->IDLE");

            break;
        }
    }
}
