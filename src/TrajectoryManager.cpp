#include "TrajectoryManager.h"

TrajectoryManager::
TrajectoryManager( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    world_frame_id_("world"),
    has_target_(false),
    vesc_ready_(false),
    yaw_ready_(false),
    state_(StateEnum::IDLE),
    trigger_time_(ros::Time::now()),
    cmd_sent_time_(ros::Time::now())
{
    nh_ = nh;
    
    // Pull down params
    nh_.param<double>("launch_angle",launch_angle_deg_, 30.0);
    nh_.param<double>("max_yaw_angle", max_yaw_angle_,  85.f);
    nh_.param<double>("min_yaw_angle", min_yaw_angle_, -85.f);
    nh_.param<double>("max_initial_velocity", max_initial_velocity_, 1000);
    nh_.param<bool>("plot_traj", plot_traj_ , true);
    nh_.param<bool>("plot_target", plot_target_ , true);

    // Initialize duration params
    double cmd_timeout_sec; 
    nh_.param<double>("command_timeout",cmd_timeout_sec, 30.f);
    cmd_timeout_ =  ros::Duration(cmd_timeout_sec);
    double cooldown_time_sec; 
    nh_.param<double>("cooldown_time",cooldown_time_sec, 3.f); 
    cooldown_time_ =  ros::Duration(cooldown_time_sec);

    // Setup subscribers
    trajectory_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("target_pose", 1, &TrajectoryManager::trajectoryPoseCallback, this);
    vesc_ready_sub_ = nh_.subscribe<std_msgs::Bool>("vesc_ready", 1, &TrajectoryManager::vescReadyCallback, this);
    yaw_ready_sub_ = nh_.subscribe<std_msgs::Bool>("yaw_ready", 1, &TrajectoryManager::yawReadyCallback, this);
    abort_sub_ = nh_.subscribe<std_msgs::Empty>("abort", 1, &TrajectoryManager::abortCallback, this);

    // Setup publishers
    vesc_cmd_pub_= nh.advertise<std_msgs::Float32>("velocity_cmd", 1);
    yaw_cmd_pub_= nh.advertise<std_msgs::Int8>("yaw_cmd", 1);
    trigger_pub_= nh.advertise<std_msgs::Empty>("trigger", 1);
    shot_pub_ = nh.advertise<geometry_msgs::PoseStamped>("shot",1);
    state_pub_ = nh.advertise<std_msgs::Int8>("trajectory_manager_state",1);

    // Initialize visualization
    if (plot_target_)
    {
        target_pub_ = nh.advertise<visualization_msgs::Marker>("launcher_target",1);
    }
    if (plot_traj_)
    {
        trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory",1);
    }
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
        bool yaw_in_range = min_yaw_angle_ < target_yaw_ && target_yaw_ < max_yaw_angle_;
        bool v_in_range = target_velocity_ >= 0 && target_velocity_ < max_initial_velocity_;

        if ( yaw_in_range && v_in_range )
        {
            has_target_ = true;
            abort_ = false; // overwrite abort signal if present

            if (plot_target_)
            {
                target_pub_.publish(buildTargetMarker());
            }
            if (plot_traj_)
            {
                trajectory_pub_.publish(buildTrajectoryMarker());
            }

        }
        else
        {
            // Reject invalid command and log the issue
            has_target_ = false;

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
    double yaw = atan2(target_pose->pose.position.y,target_pose->pose.position.x) * 180/M_PI;
    return yaw;
}

float
TrajectoryManager::
calculateInitialVelocity(const geometry_msgs::PoseStamped::ConstPtr& target_pose )
{
    // Check if transform to world frame is available
    double launcher_pitch = 0.0;
    if( tf_buffer_.canTransform( target_pose->header.frame_id, world_frame_id_, ros::Time::now(), ros::Duration(3.0) ) )
    {

        // Extract the pitch from the transform
        double roll, pitch, yaw;
        geometry_msgs::TransformStamped world_2_launcher = tf_buffer_.lookupTransform( target_pose->header.frame_id, world_frame_id_, ros::Time::now(), ros::Duration(0.0) );
        tf2::Quaternion launcher_orientation;
        tf2::convert(world_2_launcher.transform.rotation, launcher_orientation);
        tf2::Matrix3x3 m(launcher_orientation);
        m.getRPY(roll, pitch, yaw);
        launcher_pitch = pitch * (180.f / M_PI);

        ROS_INFO("World transform available, launcher pitch: %f", launcher_pitch);
    }

    // Calculate the initial velocity of the ball neglecting drag
    double d = sqrt( pow(target_pose->pose.position.x, 2) + pow(target_pose->pose.position.y, 2) );
    double z_c = target_pose->pose.position.z;
    double theta = (launch_angle_deg_ + launcher_pitch) * (M_PI / 180.f);
    
    // Calculate time the ball hits the cup
    contact_time_ = sqrt( ( 2.f * ( d * tan(theta) - z_c ) ) / G );
    double launch_v = d / ( cos(theta) * contact_time_ );

    return launch_v;
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

visualization_msgs::Marker
TrajectoryManager::
buildTargetMarker()
{
    geometry_msgs::Quaternion identity;
    identity.w = 1.f;

    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = target_pose_.header.frame_id;
    target_marker.header.stamp = ros::Time();
    target_marker.ns = "launcher_target";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose = target_pose_.pose;
    target_marker.pose.orientation = identity;
    target_marker.scale.x = 0.1;
    target_marker.scale.y = 0.1;
    target_marker.scale.z = 0.1;
    target_marker.color.a = 1.0;
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 0.0;

    return target_marker;
}

visualization_msgs::Marker
TrajectoryManager::
buildTrajectoryMarker()
{
    geometry_msgs::Quaternion identity;
    identity.w = 1.f;

    // Create marker
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = target_pose_.header.frame_id;
    trajectory_marker.header.stamp = ros::Time();
    trajectory_marker.ns = "launcher_traj";
    trajectory_marker.id = 0;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.pose.orientation = identity; // TODO: make identity quaternion
    trajectory_marker.scale.x = 0.1;
    trajectory_marker.scale.y = 0.1;
    trajectory_marker.scale.z = 0.1;
    trajectory_marker.color.a = 1.0;
    trajectory_marker.color.r = 0.0;
    trajectory_marker.color.g = 0.0;
    trajectory_marker.color.b = 1.0;

    // Initialize time vector
    int num_steps = 100;
    double step_size = (contact_time_)/num_steps; 

    // Cache some important values
    float r2y = sin(target_yaw_ * M_PI/180.f);
    float r2x = cos(target_yaw_ * M_PI/180.f);
    float r_dot = target_velocity_ * cos(launch_angle_deg_ * M_PI/180.f);
    float z_dot = target_velocity_ * sin(launch_angle_deg_ * M_PI/180.f);

    // Loop over time interval
    for(int i = 0; i <= num_steps; i++)
    {
        // Calculate point along trajectory
        double t = i * step_size; 
        float r = r_dot * t; 
        float z = z_dot * t - (0.5 * G * pow(t,2)); 

        geometry_msgs::Point point;
        point.x = r * r2x;
        point.y = r * r2y;
        point.z = z;
        trajectory_marker.points.push_back(point);
    }

    return trajectory_marker;
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

