#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to get encoder values read by the Raspberry Pi through a Linux Kernel
Module and publish them to /encoder.  
"""

import numpy as np
import rospy
import traceback
import encoders
from std_msgs.msg import (
    Int32,
    Int32MultiArray,
)


rospy.init_node("encoder_reading_node", anonymous=False)


def talker():
    pub_enc = rospy.Publisher("/encoder", Int32MultiArray, queue_size=1)

    msg_enc = Int32MultiArray()

    # Set update frequency (Hz)
    rosTimer = rospy.Rate(60)

    while not rospy.is_shutdown():
        # read the encoders:
        [rightEnc, leftEnc] = encoders.readEncoders()  # They return as Ints

        # Pack encoder data into message and publish
        msg_enc.data = [
            leftEnc,
            rightEnc,
        ]
        pub_enc.publish(msg_enc)

        rosTimer.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        # Print traceback if an error happens
        traceback.print_exc()
        pass
