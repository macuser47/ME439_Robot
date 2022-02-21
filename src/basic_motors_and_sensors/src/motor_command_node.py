#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to get User Input of commands for wheel command and publish them to
/wheel_command to set wheel commands.
"""

import rospy
from std_msgs.msg import Float32
from basic_motors_and_sensors.msg import WheelCommands

rospy.init_node("motor_command_node", anonymous=False)


def talker_for_wheel_commands():

    # Set up a Publisher
    pub_wheel_command_left = rospy.Publisher(
        "wheel_command", WheelCommands, queue_size=1
    )

    while not rospy.is_shutdown():

        wheel_command_left = int(
            input("Enter wheel command left, right (-480 to +480) \n")
        )
        wheel_command_right = int(
            input("Enter wheel command left, right (-480 to +480) \n")
        )

        # Check for good inputs and fix them if bad:
        if wheel_command_left < -480:
            wheel_command_left = -480
        elif wheel_command_left > 480:
            wheel_command_left = 480

        if wheel_command_right < -480:
            wheel_command_right = -480
        elif wheel_command_right > 480:
            wheel_command_right = 480

        # Pack the message object and publish
        wheel_command_msg = WheelCommands(
            left=wheel_command_left, right=wheel_command_right
        )
        pub_wheel_command_left.publish(wheel_command_msg)


if __name__ == "__main__":
    try:
        talker_for_wheel_commands()
    except rospy.ROSInterruptException:
        pass
