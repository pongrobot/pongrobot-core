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

    def __init__(self, port1_name, port2_name, timeout=10, rate=10):
        # ROS data
        self.duty_cycle_sub = rospy.Subscriber("duty_cycle_cmd", Float32, self.duty_cycle_callback)
        self.rpm_sub = rospy.Subscriber("rpm_cmd", Float32, self.rpm_callback)
        self.vel_sub = rospy.Subscriber("velocity_cmd", Float32, self.vel_callback)
        self.trigger_sub = rospy.Subscriber("trigger", Empty, self.trigger_callback)
        self.ready_pub = rospy.Publisher("vesc_ready", Bool, queue_size=10)
        self.rate = rospy.Rate(10) 

        # Port data
        self.port1_name = port1_name
        self.port2_name = port2_name
        self.port1_open = False
        self.port2_open = False
        self.open_port1()
        self.open_port2()

        # Command Data
        self.duty_cycle = 0
        self.rpm = 0
        self.command_mode = CommandMode.NO_COMMAND
        self.last_command_time = rospy.get_rostime()
        self.command_timeout = timeout

        # Motor data
        self.ramp_time = 3 # time it takes for vesc to get up to speed (sec)
        self.cooldown_time = 5 # time to wait after trigger to shut down motor (sec)
        self.wheel_radius = 0.3 # radius of launcher wheel in meters
        self.at_setpoint = False
        self.cooling_down = False
        self.trigger_time = rospy.get_rostime()

    def duty_cycle_callback(self, msg):
        self.duty_cycle = msg.data
        self.command_mode = CommandMode.DUTY_CYCLE_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        rospy.loginfo('Received DUTY_CYCLE_COMMAND: ' + str(self.duty_cycle) + '%')

    def rpm_callback(self, msg):
        self.rpm = msg.data 
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        rospy.loginfo('Received RPM_COMMAND: ' + str(self.rpm) + ' rpm')

    def vel_callback(self, msg):
        self.rpm = self.velocity_to_rpm(msg.data)
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        rospy.loginfo('Received VELOCITY_COMMAND: ' + str(msg.data) + ' m/s')

    def trigger_callback(self, msg):
        if self.command_mode != CommandMode.NO_COMMAND:
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
        # TODO: Apply acceleration curve
        if self.port1_open and self.port2_open:
            rospy.loginfo('Sending DUTY_CYCLE_COMMAND = ' + str(self.duty_cycle))
            self.port1.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle) * 1000) )) )
            self.port2.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle) * 1000) )) )

    def send_rpm_command(self):
        # TODO: Apply acceleration curve
        if self.port1_open and self.port2_open:
            rospy.loginfo('Sending RPM_COMMAND = ' + str(self.rpm))
            self.port1.write( pyvesc.encode( pyvesc.SetRPM( int(self.rpm * 7)) ) )
            self.port2.write( pyvesc.encode( pyvesc.SetRPM( int(self.rpm * 7)) ) )

    def velocity_to_rpm(self, v):
        # TODO: Calibrate experimentally and add a calibration function
        rpm = v * 30.0/(self.wheel_radius * math.pi)
        return rpm 

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if ~self.port1_open:
                self.open_port1()

            if ~self.port2_open:
                self.open_port2()

            # Check if we have a valid command
            if self.command_mode != CommandMode.NO_COMMAND:
                if (rospy.get_rostime() - self.last_command_time).to_sec() > self.command_timeout:
                    # Command has timed out, shut down motor
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND

                elif self.cooling_down and (rospy.get_rostime() - self.trigger_time).to_sec() > self.cooldown_time:
                    # ball has been launched, shut down motor
                    self.cooling_down = False
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND
                    rospy.loginfo('Cooled down after trigger')

                else:
                    self.at_setpoint = (rospy.get_rostime() - self.last_command_time).to_sec() > self.ramp_time
                    if self.command_mode == CommandMode.DUTY_CYCLE_COMMAND:
                        self.send_duty_cycle_command()
                    elif self.command_mode == CommandMode.RPM_COMMAND:
                        self.send_rpm_command()

            # Report status 
            self.ready_pub.publish(Bool(self.at_setpoint))
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('number_counter')
    vesc_handler = VescHandler('/dev/ttyACM0', '/dev/ttyACM1', 30)
    vesc_handler.run()
    
