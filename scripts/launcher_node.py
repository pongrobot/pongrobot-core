import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from enum import Enum
import math
import serial
import sys


class LauncherHandler:
    def __init__(self, port1):
        # ROS subscribers/publishers
        self.yaw_cmd_sub = rospy.Subscriber("yaw_cmd", Float32, self.yaw_cmd_callback)
        self.trigger_sub = rospy.Subscriber("trigger", Empty, self.trigger_callback)
        self.reset_sub   = rospy.Subscriber("reset", Empty, self.reset_callback)
        self.serial_console_sub   = rospy.Subscriber("serial_console_in", Empty, self.serial_console_callback)

        self.yaw_ready_pub = rospy.Publisher("yaw_ready", Bool, queue_size=10)
        self.has_ball_pub = rospy.Publisher("has_ball", Bool, queue_size=10)
        self.serial_console_pub = rospy.Publisher("serial_console_out", String, queue_size=10)

        self.load_config()

        self.launcher_yaw_offset = 5.0
        self.yaw_ready_val = 0
        self.ball_ready_val = 0

        # Port data
        self.port_name = port1
        self.port_open = False
        self.open_port()

        # State
        self.heartbeat_raw = None
        self.heartbeat_split = [None,None,None,None,None,None,None]
        self.serial_buffer = ""

    def load_config(self):
        self.rate = rospy.Rate( rospy.get_param("/rate/launcher") ) 

    def serial_console_callback(self, msg):
        rospy.loginfo("Received SERIAL command: {}".format(msg.data))
        if self.port_open:
            self.port.write("{}\n".format(msg))
            self.port.flush()

    def yaw_cmd_callback(self, msg):
        self.launcher_yaw_offset = rospy.get_param("yaw/fudge_offset")
        self.launcher_yaw_scalar = rospy.get_param("yaw/fudge_scalar")

        yaw_val = (msg.data * self.launcher_yaw_scalar) + self.launcher_yaw_offset
        rospy.loginfo("Received YAW command: {:.4f} deg".format(yaw_val))
        if self.port_open:
            self.port.write(("m,{}\n".format(yaw_val)).encode(encoding='utf-8'))
            self.port.flush()

    def trigger_callback(self, msg):
        rospy.loginfo('Received TRIGGER command')
        self.port.write("l\n".encode(encoding='utf-8'))
        self.port.flush()

    def reset_callback(self, msg):
        rospy.loginfo('Received RESET command')
        self.port.write("r\n".encode(encoding='utf-8'))
        self.port.flush()
        self.port.write("z,start\n".encode(encoding='utf-8'))
        self.port.flush()

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
                    data_str = self.port.read(self.port.inWaiting()).decode('ascii')
                    for character in data_str:
                        if character != '\n':
                            self.serial_buffer += character
                        else: 
                            # Republish
                            self.serial_console_pub.publish(String(self.serial_buffer))
                            
                            # Parse heartbeat if we receive a newline
                            raw_msg = self.serial_buffer.strip()
                            try:
                                split_potential = raw_msg.split(',')
                                if len(split_potential) == 7:
                                    self.heartbeat_raw = raw_msg
                                    self.heartbeat_split = split_potential
                                    rospy.loginfo("Received heartbeat: [{}]".format(self.heartbeat_raw))

                            except Exception as e:
                                rospy.loginfo("Skipping message: [{}]".format(raw_msg))
                                rospy.logerr("Error:",e)
                                pass
                            # Clear
                            self.serial_buffer = ""

            if self.heartbeat_raw is not None:
                if self.yaw_ready_val != int(self.heartbeat_split[5]):
                    self.yaw_ready_val = int(self.heartbeat_split[5])
                    self.yaw_ready_pub.publish(Bool(self.yaw_ready_val == 1))
                
                if self.ball_ready_val != int(self.heartbeat_split[6]):
                    self.ball_ready_val = int(self.heartbeat_split[6])
                    self.has_ball_pub.publish(Bool(self.ball_ready_val == 1))
            else:
                if self.yaw_ready_val != 0:
                    self.yaw_ready_pub.publish(Bool(False))
                    self.yaw_ready_val = 0
                
                if self.ball_ready_val != 0:
                    self.has_ball_pub.publish(Bool(False))
                    self.ball_ready_val = 0
    
            self.rate.sleep()


if __name__ =='__main__':
    rospy.init_node('launcher_node')
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) >= 1:
        launcher_handler = LauncherHandler(sys.argv[1])
        launcher_handler.run()
    else:
        print("Invalid number of args")

    
