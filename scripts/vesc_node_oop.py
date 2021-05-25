import rospy
from std_msgs.msg import Float64
from enum import Enum
import serial

class CommandMode(Enum):
    NO_COMMAND=0
    DUTY_CYCLE_COMMAND=1
    RPM_COMMAND=2

class VescHandler:

    def __init__(self, port_name, rate=10, timeout=10):
        # ROS data
        self.duty_cycle_sub = rospy.Subscriber("duty_cycle", Float64, self.duty_cycle_callback)
        self.rpm_sub = rospy.Subscriber("rpm", Float64, self.rpm_callback)
        self.rate = rospy.Rate(10) 

        # Port data
        self.port_name = port_name
        self.port_open = False
        self.open_port()

        # Command Data
        self.duty_cycle = 0
        self.rpm = 0
        self.command_mode = CommandMode.NO_COMMAND
        self.last_command_time = rospy.get_rostime()
        self.command_timeout = 10

    def duty_cycle_callback(self, msg):
        self.duty_cycle = msg.data
        self.command_mode = CommandMode.DUTY_CYCLE_COMMAND
        rospy.loginfo('Received DUTY_CYCLE_COMMAND: ' + str(self.duty_cycle))

    def rpm_callback(self, msg):
        self.rpm = msg.data 
        self.command_mode = CommandMode.RPM_COMMAND
        rospy.loginfo('Received RPM_COMMAND: ' + str(self.rpm))

    def open_port(self):
        try:
            self.port = serial.Serial(self.port_name)
            self.port_open = True
        except:
            rospy.logerr('Unable to open port:' + self.port_name) 
            self.port_open = False

    def send_duty_cycle_command(self):
        if self.port_open:
            rospy.loginfo('Sending DUTY_CYCLE_COMMAND = ' + str(self.duty_cycle))
            self.port.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle) * 1000) )) )

    def send_rpm_command(self):
        if self.port_open:
            rospy.loginfo('Sending RPM_COMMAND = ' + str(self.rpm))
            port.write( pyvesc.encode( pyvesc.SetRPM( int(self.rpm * 7)) ) )

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if ~self.port_open:
                self.open_port()

            # Check if we have a valid command
            if self.command_mode != CommandMode.NO_COMMAND:
                if (rospy.get_rostime() - self.last_command_time).to_sec() > self.command_timeout:
                    self.command_mode = CommandMode.NO_COMMAND
                else:
                    if self.command_mode == CommandMode.DUTY_CYCLE_COMMAND:
                        self.send_duty_cycle_command()
                    elif self.command_mode == CommandMode.RPM_COMMAND:
                        self.send_rpm_command()

            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('number_counter')
    vesc_handler = VescHandler('/dev/ttyACM0')
    vesc_handler.run()
    
