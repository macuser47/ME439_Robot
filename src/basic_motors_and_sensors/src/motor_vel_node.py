#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Velocity Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_node.py
ROS Node to accept commands of "wheel_command_left" (left wheel only at first) and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from pololu_drv8835_rpi import motors, MAX_SPEED  # MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place.
rospy.init_node("motor_vel_node", anonymous=False)


def listener():

    # Subscribe to the "wheel_command_left" topic
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    sub = rospy.Subscriber("/wheel_speeds", Float32MultiArray, set_wheel_command)

    rospy.spin()  # keep the node from exiting


# Callback for actually setting the command of the left wheel
def set_wheel_command(msg_in):
    wheel_command_left, wheel_command_right = int(msg_in.data[0]), int(msg_in.data[1])
    print(f"Got wheel commands l:{wheel_command_left} r:{wheel_command_right}")
    motors.setSpeeds(wheel_command_left, wheel_command_right)


#### CODE HERE ####
# Add a callback for the Right wheel
#### END CODE ####


# Section to start the execution, with Exception handling.
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass

    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
