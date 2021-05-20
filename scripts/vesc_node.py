#!/usr/bin/env python

import rospy
import pyvesc
import serial
from std_msgs.msg import Float64

duty_cycle = 0
rpm = 0
command_mode = 0
command_time = 0 

def duty_cycle_callback(data):
    global duty_cycle, command_mode, command_time
    duty_cycle = data.data
    command_mode = 1
    command_time = rospy.get_rostime() 
    rospy.logwarn('Received Duty Cycle Command: ' + str(duty_cycle))

def rpm_callback(data):
    global rpm, command_mode, command_time
    rpm = data.data
    command_mode = 2
    command_time = rospy.get_rostime() 
    rospy.logwarn('Received RPM Command: ' + str(rpm))

def vesc_node():
    global duty_cycle, rpm, command_mode, command_time
    
    # initialize ROS data
    rospy.init_node('vesc_node')
    command_time = rospy.get_rostime()
    rate = rospy.Rate(10) 

    rospy.Subscriber('duty_cycle', Float64, duty_cycle_callback)
    rospy.Subscriber('rpm', Float64, rpm_callback)

    # intialize serial connection
    port = serial.Serial('/dev/ttyACM0')
    port2 = serial.Serial('/dev/ttyACM1')
    COMMAND_TIMEOUT = 10 # timeout in seconds

    while not rospy.is_shutdown():
        # send outgoing command 
        if command_mode != 0:
            if (rospy.get_rostime() - command_time).to_sec() > COMMAND_TIMEOUT:
                command_mode = 0
            else:
                if command_mode == 1:
                    # send duty cycle command to vesc
                    rospy.logwarn('Active Duty Cycle Command: ' + str(duty_cycle))
                    if port.isOpen():
                        port.write( pyvesc.encode( pyvesc.SetDutyCycle( int((duty_cycle) * 1000) )) )
                    if port2.isOpen():
                        port2.write( pyvesc.encode( pyvesc.SetDutyCycle( int((duty_cycle) * 1000) )) )
                elif command_mode == 2:
                    # send rpm command to vesc
                    rospy.logwarn('Active RPM Command: ' + str(rpm))
                    if port.isOpen():
                        port.write( pyvesc.encode( pyvesc.SetRPM( int(rpm * 7)) ) )
                    if port2.isOpen():
                        port2.write( pyvesc.encode( pyvesc.SetRPM( int(rpm * 7)) ) )

        rate.sleep()

if __name__ == "__main__":
    try:
        vesc_node()
    except rospy.ROSInterruptException:
        pass
