# pongrobot_actuation 1.0.0
ROS package to handle all the trajectory calculations and launcher functions for thr brobot pongrobot. This includes calculating a trajectory from a target location, generating the required yaw and initial velocity commands and coordinating the shot with the hardware.This tag should be run against pongrobot_perception 1.0.0 for best compatibility.

## Dependencies
- rosserial
- tf2
- PySerial
- [PyVESC](https://github.com/LiamBindle/PyVESC)
### Arduino Dependencies
- rosserial-arduino
- AccelStepper
  
### A Note on PySerial
For many systems another python package 'serial' will cause problems when `import serial` is called so often serial must be uninstalled and pyserial must be installed in its place.
Uninstall serial: `pip3 uninstall serial`
Install pyserial: `pip3 install pycrc`

### A Note on PyVesc
When installing PyVesc, often the wrong pycrc package will be installed ( pycrc vs PyCRC). The all lowercase pycrc should be the correct package so if there are import errors while running pyvesc, remove the bad package and install the correct one.
Install pyvesc: `pip3 install pyvesc`
Uninstall PyCRC: `pip3 uninstall PyCRC`
Install pycrc: `pip3 install pycrc`

## Node Descriptions
- `vesc_node` - the only python node on the robot, used as an interface to the motor controllers. Handles commands in the form of RPM, duty cycle and ball speed (m/s) RPM calibration is handled here. Takes two args which are the port names for each of the two VESCs
- `trajectory_manager_node` - takes in the target position from the perception system, calculate the trajectory and generate the initial velocity and yaw commands for the hardware interface nodes. This node will also coordinate the trigger signal when the yaw and motors report that they are ready.
- `launcher_node` - and arduino node used to control the yaw setpoint, trigger and sense if there is a ball in the launcher. The node is connected over rosserial.

## Topics
- `/launcher/target_pose [geometry_msgs/PoseStamped]` - the topic for the target cup to hit, (published in launcher frame)
- `/launcher/vesc_ready [std_msgs/Bool]` - show if the vesc is at the setpoint it was commanded
- `/launcher/yaw_ready [std_msgs/Bool]` - show if the yaw is at the setpoint it was commanded
- `/abort [std_msgs/Empty]` - signal to abort the current trajectory command at whatever stage it's in
- `/launcher/velocity_cmd [std_msgs/Float32]` - the initial velocity command sent to the vesc node in m/s
- `/launcher/rpm_cmd [std_msgs/Float32]` - the RPM command sent to the vesc node in m/s
- `/launcher/duty_cycle_cmd [std_msgs/Float32]` - the duty cycle command sent to the vesc node in %
- `/launcher/yaw_cmd [std_msgs/Int8]` - the angle setpoint command to the yaw node in degrees
- `/launcher/trigger [std_msgs/Empty]` - signal for the launcher node to activate the servo and shoot the ball
- `/launcher/target_pose [geometry_msgs/PoseStamped]` - signal sent from the launcher to confirm that a shot has completed. The location of the shot is included to discriminate between commands
- `/launcher/trajectory_manager_state [std_msgs/Int8]` - report the current state enum of the trajectory manager
- `/launcher/has_ball [std_msgs/Bool]` - signal from the launcher telling if a ball is present. Used by the Game Manager to trigger a shot

## Config Options
Node rate config options are available in `config/rate_config.yaml` and loaded under the namespace `/rate`
- `trajectory`: 20.0 # rate to run the TrajectoryManager node (hz)
- `vesc`: 20.0 # loop rate for the VESC node (hz)
  
Launcher specific config options are available in `config/launcher_config.yaml` and loaded under the namespace `/launcher`
- `geometry/launch_angle`: fixed angle of the launcher (deg)
- `geometry/wheel_radius`: radius of the launcher wheel in m
- `trajectory/min_yaw_angle`: min allowable yaw angle command (deg)
- `trajectory/max_yaw_angle`: max allowable yaw angle command (deg)
- `trajectory/max_initial_velocity`: max allowable velocity command (m/s)
- `trajectory/target_frame`: coordinate frame to use if the transform is available
- `visualization/plot_traj`: enable plotting rviz markers of the planned trajectory
- `visualization/plot_target`: enable plotting an rviz marker for the target pose
- `vesc/num_motor_poles`: number of motor poles
- `vesc/rpm_accel`: rpm acceleration limit in rpm/sec
- `vesc/duty_cycle_accel`: duty cycle acceleration limit in %/sec
- `vesc/ramp_time`: maximum time it take for the vesc to reach a commanded value (sec)
- `vesc/cooldown_time`: time after a trigger command has been send before the vescs turn off (sec)
- `vesc/command_timeout`: max time a command will stay active waiting for a trigger signal (sec) 
- `vesc/max_rpm`: max rpm value for the vesc
- `vesc/rpm_calibration_slope`: RPM calibration slope from regression analysis
- `vesc/rpm_calibration_offset`: RPM calibration offset from regression analysis
- `vesc/fudge`: fudge factor for motor commands, should be driven to zero with a well calibrated system

## Provided launchfiles
- `launcher.launch`: Run all the nodes in the package including the VESC node, the TrajectoryManager and connecting to the launcher node over rosserial
- `launcher_headless.launch`: run all the same nodes as `launcher.launch` without the rviz visualization
- `manager.launch`: Launch only the trajectory manager node, used for debugging and development of the trajectory calculation

## Utilities
Some useful utilities are provided under `/utils`, these are used for setting up the various udev rules.

### Udev Rules
Before the package can be run, the appropriate udev rules must be setup. This will allow the system to correctly identify USB devices. To setup the rules for the vescs and the launcher node, copy the contents of `/utils` to `/etc/udev/rules.d` by running the following command from the package root directory `sudo cp utils/* /etc/udev/rules.d`. You may need to update the execution permissions of `vesc_count`. After the rules have been installed, make sure the machine is rebooted or the udev rules are reloaded.

## Reference Frames
- World Frame: the origin is on the ground at the base of the center of the robot, Z is normal to the ground
- Robot Base Frame: The same origin as World Frame with pitch and roll attitude of the robot calculated from the IMU. __NOTE: yaw component is removed from the transform__
- Robot Center Frame: the same attitude as Robot Base frame but the origin is translated up in Z to align with the base of the robots aluminum frame
- Launcher Frame: reference frame centered at the center of the launcher. Z offset from Robot Center is defined in the config
- Camera Frame: reference frame of the camera, transform to Robot Center frame determined from config and IMU data
- Image Frame: image reference with z depth

## Trajectory Testing (with graphics)
1. Navigate to your catkin_ws
2. Make sure your code is build and the workspace has be correctly sourced (`catkin_make` & `. source devel/setup.zsh`)
3. Launch the trajectory manager launchfile `roslaunch pongrobot_actuation manager.launch`, rviz should open with the trajectory visualization config
4. In a new shell, run `rostopic echo /launcher/velocity_cmd` to subscribe to the velocity output command
5. In yet another shell use rostopic pub to send a target command to `/launcher/target_pose`, make sure to set the frame_id to `launcher` and edit the x&y position as desired.
6. The TrajectoryManager should log messages showing it has received the command and calculated the correct velocity. Rviz should have rendered the target pose with trajectory, and the velocity command subscriber from step 4 should echothe calculated velocity
7. To run with drag, replace the command from step 3 with `roslaunch pongrobot_actuation manager.launch use_drag:=true`

**Note**: If your system does not have graphics setup, you can disable the rviz trajectory visualization by running the manager launchfile with the `use_graphics:=false` argument
