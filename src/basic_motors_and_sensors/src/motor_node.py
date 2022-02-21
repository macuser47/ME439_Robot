#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to accept commands of "wheel_command" and run motors using the Pololu
DRV8835 Raspberry Pi Hat 
"""

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from pololu_drv8835_rpi import motors, MAX_SPEED  # MAX_SPEED is 480 (hard-coded)

rospy.init_node("motor_node", anonymous=False)


def listener():
    sub = rospy.Subscriber("/wheel_command", Float32MultiArray, set_wheel_command)
    rospy.spin()  # keep the node from exiting


def set_wheel_command(msg_in):
    """Set wheel voltages when a new command comes in"""
    wheel_command_left, wheel_command_right = int(msg_in.data[0]), int(msg_in.data[1])
    print(f"Got wheel commands l:{wheel_command_left} r:{wheel_command_right}")
    motors.setSpeeds(-wheel_command_left, wheel_command_right)


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass

    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
