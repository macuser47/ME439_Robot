#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to set wheel commands based on sensor data.
"""

import rospy
from std_msgs.msg import Float32
from basic_motors_and_sensors.msg import WheelCommands

rospy.init_node("motor_command_node", anonymous=False)
pub_wheel_command = rospy.Publisher("/wheel_command", WheelCommands, queue_size=1)


def talker_for_wheel_commands():
    ultrasonic_sub = rospy.Subscriber("/ultrasonic_proc", Float32, sensor_move)
    rospy.spin()


def sensor_move(ultrasonic_msg):
    print(f"Got msg: {ultrasonic_msg.data}")
    if ultrasonic_msg.data > 0.2:
        wheel_command_left = 480
        wheel_command_right = 480
    else:
        wheel_command_left = -480
        wheel_command_right = -480

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
    wheel_command_msg = WheelCommands(
        left=wheel_command_left, right=wheel_command_right
    )
    pub_wheel_command.publish(wheel_command_msg)


if __name__ == "__main__":
    try:
        talker_for_wheel_commands()
    except rospy.ROSInterruptException:
        # Stop the wheels when interrupted
        wheel_command_msg = WheelCommands(left=0, right=0)
        pub_wheel_command.publish(wheel_command_msg)
        pass
