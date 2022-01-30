#include "TrajectoryManager.h"

TrajectoryManager::
TrajectoryManager( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    has_target_(false),
    vesc_ready_(false),
    yaw_ready_(false),
    state_(StateEnum::IDLE),
    trigger_time_(ros::Time::now()),
    cmd_sent_time_(ros::Time::now())
{
    nh_ = nh;
    
    load_params();    

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
        if (use_drag_)
        {
            target_velocity_ = calculateInitialVelocityDrag(msg);
        } else {
            target_velocity_ = calculateInitialVelocity(msg);
        }

        ROS_WARN("Calculated velocity: %f m/s", target_velocity_);    

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
                if (use_drag_)
                {
                    trajectory_pub_.publish(buildTrajectoryMarkerDrag());                   
                } else {
                    trajectory_pub_.publish(buildTrajectoryMarker());
                }
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

        ROS_INFO("[TrajectoryManager] World transform available, launcher pitch: %.4f deg", launcher_pitch);
    }

    // Calculate the initial velocity of the ball neglecting drag
    double d = sqrt( pow(target_pose->pose.position.x, 2) + pow(target_pose->pose.position.y, 2) );
    double z_c = target_pose->pose.position.z;
    double theta = (launch_angle_deg_ + launcher_pitch) * (M_PI / 180.f); // launch angle relative to gravity
    
    // Calculate time the ball hits the cup
    contact_time_ = sqrt( ( 2.f * ( d * tan(theta) - z_c ) ) / G );
    double launch_v = d / ( cos(theta) * contact_time_ );

    return launch_v;
}

float TrajectoryManager::calculateInitialVelocityDrag(const geometry_msgs::PoseStamped::ConstPtr& target_pose)
{
    // Check if transform to world frame is available
    float launcher_pitch = 0.0;
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

        ROS_INFO("[TrajectoryManager] World transform available, launcher pitch: %.4f deg", launcher_pitch);
    }

    // Calculate the initial velocity of the ball neglecting drag
    float d = sqrt( pow(target_pose->pose.position.x, 2) + pow(target_pose->pose.position.y, 2) );
    float z_c = target_pose->pose.position.z;
    float theta = (launch_angle_deg_ + launcher_pitch) * (M_PI / 180.f); // launch angle relative to gravity

    // Calculate time the ball hits the cup
    contact_time_ = sqrt( ( 2.f * ( d * tan(theta) - z_c ) ) / G ); // This is only for the trajectory Marker Builder

    // initial velocity guess
    float v0_guess = d / ( cos(theta) * contact_time_ );
    Eigen::VectorXf x(1);
	x(0) = v0_guess;             

    LMFunctor_DragError functor;
    functor.target_pose = target_pose->pose;
    functor.theta = theta;
    functor.m = 3; // 3 axes of the position error
	functor.n = 1; // just one initial velocity

    Eigen::LevenbergMarquardt<LMFunctor_DragError, float> lm(functor);
	int status = lm.minimize(x);
	std::cout << "LM optimization status: " << status << std::endl;

    // Initializing the position array
    trajectory_pose_array_=functor.get_trajectory_pose_array();
    trajectory_pose_array_.header.stamp = ros::Time();

    //float launch_v = fsolve(@(v0) calcDragError(target_pose,v0,theta),v0_guess)


    return x(0);
}

void TrajectoryManager::LMFunctor_DragError::set_position_array(const Eigen::Matrix<float, 3, Eigen::Dynamic>& pos_matrix) {
    position_array = pos_matrix;
}
Eigen::Matrix<float, 3, Eigen::Dynamic> TrajectoryManager::LMFunctor_DragError::get_position_array() const{
    return position_array;
}

// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
int TrajectoryManager::LMFunctor_DragError::operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) 
{
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.
    // Only 1 parameter, which is v0.

    // 'fvec' has dimensions m x 1
    // It will contain the error for each data point.
    
    Eigen::Vector3f new_pose(3);
    new_pose << 0,
                0,
                0;
    Eigen::Matrix<float, 3, Eigen::Dynamic> pos_matrix;
    pos_matrix.col(0) = new_pose;
    
    Eigen::Vector3f target_vector(3);
    target_vector <<    target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z;

    float v0 = x(0);

    // Ping pong ball stuff
    double diameter = 0.04; // m
    double radius = diameter*.5; // m
    double mass = 0.0027; // kg
    double air_density = 1.29; // kg/m^3
    double drag_coefficient = 0.5; // typically .4-.6. TODO: make this a parameter.

    double k_drag = drag_coefficient * air_density * M_PI * pow(diameter,2) / (8 * mass); //constant for drag
    
    double dt = .01; // s

    float G = 9.8; // m/s^2

    Eigen::Vector3f gravity_vector;
    gravity_vector = Eigen::Vector3f::Constant(0,0,G);
    Eigen::Vector3f old_v;
    Eigen::Vector3f new_v(v0 * cos(theta), 0, v0 * sin(theta));

    float t=0;

    int i=0;
    while (new_pose(2) > target_vector(2)) // while the ball is above the plane of the cup
    {
        old_v = new_v;

        float v_mag = old_v.norm();

        Eigen::Matrix3f k;
        k << -k_drag*v_mag*dt, 0, 0, 
            0, -k_drag*v_mag*dt, 0,
            0, 0, -k_drag*v_mag*dt;
        
        new_v = old_v + k * old_v + gravity_vector*dt;
        new_pose  += new_v*dt;
        pos_matrix.resize(pos_matrix.rows(),pos_matrix.cols()+1); // Add a column	
        pos_matrix.col(pos_matrix.cols()-1) = new_pose;
        old_v=new_v; 
        t+=dt;
    }

    // contact_time_ = t; // for the trajectory builder. Don't need this if already have all of the points.

    // error is the difference between new pose and target pose
    //float error = sqrt(pow(new_pose[0]-target_pose->pose.position.x,2) + pow(new_pose[0]-target_pose->pose.position.y,2) + pow(new_pose[0]-target_pose->pose.position.z,2));
    
    // fvec is the error 
    // looping through x,y,z to get each dimension's error.
    for (int i = 0; i < 3; i++) {
        fvec(i) = new_pose[i]-target_vector(i);
    }
    
    set_position_array(pos_matrix);

    return 0;
}

// Compute the jacobian of the errors
int TrajectoryManager::LMFunctor_DragError::df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) 
{
    // 'x' has dimensions n x 1
    // It contains the current estimates for the parameters.

    // 'fjac' has dimensions m x n
    // It will contain the jacobian of the errors, calculated numerically in this case.

    float epsilon;
    epsilon = 1e-5f;

    for (int i = 0; i < x.size(); i++) {
        Eigen::VectorXf xPlus(x);
        xPlus(i) += epsilon;
        Eigen::VectorXf xMinus(x);
        xMinus(i) -= epsilon;

        Eigen::VectorXf fvecPlus(values());
        operator()(xPlus, fvecPlus);

        Eigen::VectorXf fvecMinus(values());
        operator()(xMinus, fvecMinus);

        Eigen::VectorXf fvecDiff(values());
        fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

        fjac.block(0, i, values(), 1) = fvecDiff;
    }

    return 0;
}

int 
TrajectoryManager::LMFunctor_DragError::
values() const
{ 
    return m; 
}

int
TrajectoryManager::LMFunctor_DragError::
inputs() const
{
    return n;
}

geometry_msgs::PoseArray TrajectoryManager::LMFunctor_DragError::get_trajectory_pose_array()
{
    geometry_msgs::PoseArray pose_array;
    for (int i=1;i<position_array.cols();i++)
    {
        geometry_msgs::Pose pose_pt;
        pose_pt.position.x = position_array(0,i);
        pose_pt.position.y = position_array(1,i);
        pose_pt.position.z = position_array(2,i);
        pose_array.poses.push_back(pose_pt);
    }
    return pose_array;
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

visualization_msgs::Marker
TrajectoryManager::
buildTrajectoryMarkerDrag()
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
                ROS_INFO("[TrajectoryManager] IDLE->HAS_TARGET");
            }
            break;
        }
        case StateEnum::HAS_TARGET:
        {
            // Check for abort signal
            if ( handleAbortSignal() )
            {
                ROS_INFO("[TrajectoryManager] HAS_TARGET->ABORT");
                break;
            }

            // Check for new command
            if ( handleNewCommand() )
            {
                ROS_INFO("[TrajectoryManager] HAS_TARGET->HAS_TARGET");
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
            ROS_INFO("[TrajectoryManager] Sending Command: YAW=%.4f deg, VELOCITY=%.4f m/s", target_yaw_, target_velocity_);
            ROS_INFO("[TrajectoryManager] HAS_TARGET->WAIT");

            break;
        }
        case StateEnum::WAIT:
        {
            // Check for abort signal
            if ( handleAbortSignal() )
            {
                ROS_INFO("[TrajectoryManager] WAIT->ABORT");
                break;
            }

            // Check for new command
            if ( handleNewCommand() )
            {
                ROS_INFO("[TrajectoryManager] WAIT->HAS_TARGET");
                break;
            }

            // Check if subsystems have timed out
            if ( ros::Time::now() - cmd_sent_time_ > cmd_timeout_ )
            {
                // Transition to HAS_TARGET
                state_ = StateEnum::HAS_TARGET;
                ROS_INFO("[TrajectoryManager] WAIT->HAS_TARGET");
            }

            // Check if all subsystems are ready
            if ( vesc_ready_ && yaw_ready_ )
            {
                trigger_pub_.publish(std_msgs::Empty());
                
                // Transition to SHOOT
                trigger_time_ = ros::Time::now();
                state_ = StateEnum::SHOOT;
                ROS_INFO("[TrajectoryManager] WAIT->SHOOT");
            }

            break;
        }
        case StateEnum::SHOOT:
        {
            // Wait for launcher to cool down
            if ( (ros::Time::now() - trigger_time_) > cooldown_time_ )
            {
                // Publish shot confirmation
                shot_pub_.publish(target_pose_);

                // Transition to IDLE
                state_ = StateEnum::IDLE;
                ROS_INFO("[TrajectoryManager] SHOOT->IDLE");
            }

            break;
        }
        case StateEnum::ABORT:
        {
            // Send zero velocity command to the VESC
            std_msgs::Float32 velocity_cmd;
            velocity_cmd.data = 0.f;
            vesc_cmd_pub_.publish(velocity_cmd);

            // Transition to IDLE
            state_ = StateEnum::IDLE;
            ROS_INFO("[TrajectoryManager] ABORT->IDLE");

            break;
        }
    }
}

void
TrajectoryManager::
load_params()
{
    if ( !nh_.getParam("geometry/launch_angle",launch_angle_deg_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/geometry/launch_angle", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("trajectory/max_yaw_angle", max_yaw_angle_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/trajectory/max_yaw_angle", nh_.getNamespace().c_str());
    }
    
    if ( !nh_.getParam("trajectory/min_yaw_angle", min_yaw_angle_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/trajectory/min_yaw_angle", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("trajectory/use_drag", use_drag_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/trajectory/use_drag", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("trajectory/max_initial_velocity", max_initial_velocity_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/trajectory/max_initial_velocity", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("trajectory/target_frame",world_frame_id_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/trajectory/target_frame", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("visualization/plot_traj", plot_traj_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/visualization/plot_traj", nh_.getNamespace().c_str());
    }

    if ( !nh_.getParam("visualization/plot_target", plot_target_) )
    {
        ROS_ERROR("TrajectoryManager could not load param: %s/visualization/plot_target", nh_.getNamespace().c_str());
    }

    double cmd_timeout_sec; 
    if ( !nh_.getParam("/launcher/vesc/command_timeout", cmd_timeout_sec) )
    {
        ROS_ERROR("TrajectoryManager could not load param: /launcher/vesc/command_timeout" );
    }
    cmd_timeout_ =  ros::Duration(cmd_timeout_sec);

    double cooldown_time_sec; 
    if ( !nh_.getParam("/launcher/vesc/cooldown_time",cooldown_time_sec) )
    {
        ROS_ERROR("TrajectoryManager could not load param: /launcher/vesc/cooldown_time" );
    }
    cooldown_time_ =  ros::Duration(cooldown_time_sec);

}

