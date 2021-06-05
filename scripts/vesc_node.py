import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from enum import Enum
import math
import serial
import pyvesc

class CommandMode(Enum):
    NO_COMMAND=0
    DUTY_CYCLE_COMMAND=1
    RPM_COMMAND=2

class VescHandler:

    def __init__(self):
        # ROS data
        self.duty_cycle_sub = rospy.Subscriber("duty_cycle_cmd", Float32, self.duty_cycle_callback)
        self.rpm_sub = rospy.Subscriber("rpm_cmd", Float32, self.rpm_callback)
        self.vel_sub = rospy.Subscriber("velocity_cmd", Float32, self.vel_callback)
        self.trigger_sub = rospy.Subscriber("trigger", Empty, self.trigger_callback)
        self.ready_pub = rospy.Publisher("vesc_ready", Bool, queue_size=10)
        self.rate = rospy.Rate( rospy.get_param("vesc_loop_rate", 20) ) 

        # Port data
        self.port1_name = rospy.get_param("vesc_port_1", "/dev/ttyACM0")
        self.port2_name = rospy.get_param("vesc_port_2", "/dev/ttyACM1")
        self.port1_open = False
        self.port2_open = False
        self.open_port1()
        self.open_port2()

        # Command Data
        self.command_mode = CommandMode.NO_COMMAND
        self.last_command_time = rospy.get_rostime()
        self.command_timeout = rospy.get_param("command_timeout", 30.0)

        # Motor data
        self.ramp_time = rospy.get_param("ramp_time", 3.0) # time it takes for vesc to get up to speed (sec)
        self.cooldown_time = rospy.get_param("cooldown_time", 3.0) # time to wait after trigger to shut down motor (sec)
        self.wheel_radius = rospy.get_param("wheel_radius", 0.03) # radius of launcher wheel in meters
        self.num_motor_poles = rospy.get_param("num_poles", 14)
        self.at_setpoint = False
        self.cooling_down = False
        self.trigger_time = rospy.get_rostime()

        # RPM interface
        self.target_rpm = 0
        self.rpm_cmd = 0
        self.initial_rpm = 0
        self.RPM_ACCEL = rospy.get_param("rpm_accel", 200.0) # rpm/sec
        self.MAX_RPM = rospy.get_param("MAX_RPM", 500000)

        # Duty cycle interface
        self.target_duty_cycle = 0
        self.duty_cycle_cmd = 0
        self.initial_duty_cycle = 0
        self.DUTY_CYCLE_ACCEL = rospy.get_param("duty_cycle_accel", 2.0) # %/sec

    def duty_cycle_callback(self, msg):
        if msg.data > 100:
            self.target_duty_cycle = 100
        else:
            self.target_duty_cycle = msg.data

        self.initial_duty_cycle = self.duty_cycle_cmd
        self.command_mode = CommandMode.DUTY_CYCLE_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.target_rpm = 0
        rospy.loginfo('Received DUTY_CYCLE_COMMAND: ' + str(self.duty_cycle) + '%')

    def rpm_callback(self, msg):
        if msg.data > self.MAX_RPM:
            self.target_rpm = self.MAX_RPM
        else:
            self.target_rpm = msg.data

        self.target_rpm = msg.data 
        self.initial_rpm = self.rpm_cmd
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.duty_cycle = 0
        rospy.loginfo('Received RPM_COMMAND: ' + str(self.target_rpm) + ' rpm')

    def vel_callback(self, msg):
        self.target_rpm = self.velocity_to_rpm(msg.data)
        self.initial_rpm = self.rpm_cmd
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.duty_cycle = 0
        rospy.loginfo('Received VELOCITY_COMMAND: ' + str(msg.data) + ' m/s')

    def trigger_callback(self, msg):
        if self.command_mode != CommandMode.NO_COMMAND and not self.cooling_down:
            self.cooling_down = True
            self.trigger_time = rospy.get_rostime() 
            self.last_command_time = rospy.get_rostime() # extend command timeout
            rospy.loginfo('Recieved trigger signal, starting cooldown timer')

    def open_port1(self):
        try:
            self.port1 = serial.Serial(self.port1_name)
            self.port1_open = True
        except:
            rospy.logerr('Unable to open port1:' + self.port1_name) 
            self.port1_open = False

    def open_port2(self):
        try:
            self.port2 = serial.Serial(self.port2_name)
            self.port2_open = True
        except:
            rospy.logerr('Unable to open port2:' + self.port2_name) 
            self.port2_open = False

    def send_duty_cycle_command(self):
        # TODO: Apply decceleration curve
        if self.port1_open and self.port2_open:
            if self.duty_cycle_cmd < self.target_duty_cycle:
                self.duty_cycle_cmd = self.initial_duty_cycle + (rospy.get_rostime() - self.last_command_time).to_sec() * self.DUTY_CYCLE_ACCEL
            else:
                self.duty_cycle_cmd = self.target_duty_cycle

            rospy.loginfo('Sending DUTY_CYCLE_COMMAND = ' + str(self.duty_cycle_cmd))
            self.port1.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle_cmd) * 1000) )) )
            self.port2.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle_cmd) * 1000) )) )

    def send_rpm_command(self):
        # TODO: Apply decceleration curve
        if self.port1_open and self.port2_open:
            if self.rpm_cmd < self.target_rpm:
                self.rpm_cmd = self.initial_rpm + (rospy.get_rostime() - self.last_command_time).to_sec() * self.RPM_ACCEL
            else:
                self.rpm_cmd = self.target_rpm

            rospy.loginfo('Sending RPM_COMMAND = ' + str(self.rpm_cmd))
            self.port1.write( pyvesc.encode( pyvesc.SetRPM( int(self.rpm_cmd * self.num_motor_poles)) ) )
            self.port2.write( pyvesc.encode( pyvesc.SetRPM( int(self.rpm_cmd * self.num_motor_poles)) ) )

    def velocity_to_rpm(self, v):
        # TODO: Calibrate experimentally and add a calibration function
        rpm = v * 30.0/(self.wheel_radius * math.pi)

        # clamp value to MAX_RPM
        if rpm > self.MAX_RPM:
            rpm = self.MAX_RPM

        return rpm 

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if not self.port1_open:
                self.open_port1()

            if not self.port2_open:
                self.open_port2()

            # Check if we have a valid command
            if self.command_mode != CommandMode.NO_COMMAND:
                if (rospy.get_rostime() - self.last_command_time).to_sec() > self.command_timeout:
                    # Command has timed out, shut down motor
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND
                    self.target_rpm = 0
                    self.rpm_cmd = 0

                elif self.cooling_down and (rospy.get_rostime() - self.trigger_time).to_sec() > self.cooldown_time:
                    # ball has been launched, shut down motor
                    self.cooling_down = False
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND
                    self.target_rpm = 0
                    self.rpm_cmd = 0
                    rospy.loginfo('Cooled down after trigger')

                else:
                    self.at_setpoint = (rospy.get_rostime() - self.last_command_time).to_sec() > self.ramp_time and ( self.target_rpm == self.rpm_cmd )
                    if self.command_mode == CommandMode.DUTY_CYCLE_COMMAND:
                        self.send_duty_cycle_command()
                    elif self.command_mode == CommandMode.RPM_COMMAND:
                        self.send_rpm_command()

            # Report status 
            self.ready_pub.publish(Bool(self.at_setpoint))
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('vesc_node')
    vesc_handler = VescHandler()
    vesc_handler.run()
    
