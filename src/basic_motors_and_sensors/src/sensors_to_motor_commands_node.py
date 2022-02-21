#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to set wheel commands based on sensor data.
"""

import rospy
from std_msgs.msg import Float32, Float32MultiArray

rospy.init_node("motor_command_node", anonymous=False)
pub_wheel_command = rospy.Publisher("wheel_command", Float32MultiArray, queue_size=1)


def talker_for_wheel_commands():
    ultrasonic_sub = rospy.Subscriber("/ultrasonic_proc", Float32, sensor_move)
    rospy.spin()


def sensor_move(ultrasonic_msg):
    wheel_command_msg = Float32MultiArray()

    if ultrasonic_msg.data > 0.2:
        wheel_command_left = int(200)
        wheel_command_right = int(200)
    else:
        wheel_command_left = int(-200)
        wheel_command_right = int(-200)

    # Check for good inputs and fix them if bad:
    if wheel_command_left < -480:
        wheel_command_left = -480
    elif wheel_command_left > 480:
        wheel_command_left = 480

    if wheel_command_right < -480:
        wheel_command_right = -480
    elif wheel_command_right > 480:
        wheel_command_right = 480

    # Pack the message object and publish.
    wheel_command_msg.data = [wheel_command_left, wheel_command_right]
    pub_wheel_command.publish(wheel_command_msg)


if __name__ == "__main__":
    try:
        talker_for_wheel_commands()
    except rospy.ROSInterruptException:
        # Stop the wheels when interrupted
        wheel_command_msg = Float32MultiArray()
        wheel_command_msg.data = [0, 0]
        pub_wheel_command.publish(wheel_command_msg)
        pass
