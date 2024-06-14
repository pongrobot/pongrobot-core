import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from enum import Enum
import math
import serial
import sys


class LauncherHandler:
    def __init__(self, port1):
        # ROS subscribers/publishers
        self.yaw_cmd_sub = rospy.Subscriber("yaw_cmd", Int8, self.yaw_cmd_callback)
        self.trigger_sub = rospy.Subscriber("trigger", Empty, self.trigger_callback)
        self.reset_sub   = rospy.Subscriber("reset", Float32, self.reset_callback)

        self.yaw_ready_pub = rospy.Publisher("yaw_ready", Bool, queue_size=10)
        self.has_ball_pub = rospy.Publisher("has_ball", Bool, queue_size=10)

        self.load_config()

        # Port data
        self.port_name = port1
        self.port_open = False
        self.open_port()

        # State
        self.heartbeat_raw = None
        self.heartbeat_split = [None,None,None,None,None,None,None]
        self.serial_buffer = ""

    def load_config(self):
        pass

    def yaw_cmd_callback(self, msg):
        yaw_val = msg.data
        rospy.loginfo("Received YAW command: {:.4f} deg".format(yaw_val))

    def trigger_callback(self);
        rospy.loginfo('Received TRIGGER command')

    def reset_callback(self):
        rospy.loginfo('Received RESET command')

    def open_port(self):
        try:
            self.port = serial.Serial(self.port_name)
            self.port_open = True
        except:
            rospy.logerr('Unable to open port:' + self.port_name) 
            self.port_open = False

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if not self.port_open:
                self.open_port()
            
            if self.port_open:
                # Read heartbeat data
                if (self.port.inWaiting() > 0):
                    data_str = ser.read(self.port.inWaiting()).decode('ascii')
                    for character in data_str:
                        if character != '\n':
                            self.serial_buffer += character
                        else: 
                            # Parse heartbeat if we receive a newline
                            self.heartbeat_raw = self.serial_buffer.strip()
                            rospy.loginfo("Received heartbeat: {}".format(self.heartbeat_raw))
                            self.heartbeat_split = self.heartbeat_raw.split(',')

            if self.heartbeat is not None:
                self.yaw_ready_pub.publish(Bool(int(self.heartbeat_split[5] == 1)))
                self.has_ball_pub.publish(Bool(int(self.heartbeat_split[6] == 1)))
            else:
                self.yaw_ready_pub.publish(Bool(False))
                self.has_ball_pub.publish(Bool(False))
    
            self.rate.sleep()


if __name__ =='__main__':
    rospy.init_node('launcher_node')
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) >= 1:
        launcher_handler = LauncherHandler(sys.argv[1])
        launcher_handler.run()
    else:
        print("Invalid number of args")

    
