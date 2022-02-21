#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to a receive sensor data on raw sensor data topics and process it,
then publish the processed version to another topic
"""


import rospy
from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool, Float32MultiArray
import numpy as np

# Processed data publishers
pub_A0_proc = rospy.Publisher("/light_dark_proc", Float32, queue_size=1)
pub_A1_proc = rospy.Publisher("/ultrasonic_proc", Float32, queue_size=1)
pub_enc_proc = rospy.Publisher("/encoder_proc", Float32MultiArray, queue_size=1)


def sensors_listener():
    # Initialize the Node
    rospy.init_node("sensors_processor", anonymous=False)

    # Listeners for sensor data
    sub_A0 = rospy.Subscriber("/light_dark", Int32, cal_and_pub_A0)
    sub_A1 = rospy.Subscriber("/ultrasonic", Int32, cal_and_pub_A1)
    sub_enc = rospy.Subscriber("/encoder", Int32MultiArray, cal_and_pub_encoder)

    rospy.spin()


def cal_and_pub_A0(msg_in):
    """Compute and publish light/dark state analog reading"""
    analog_level = float(msg_in.data)
    if analog_level < 500:
        pub_A0_proc.publish(Float32(data=1))
    else:
        pub_A0_proc.publish(Float32(data=0))


def cal_and_pub_A1(msg_in):
    """Compute and publish ultrasonic distance from analog reading"""
    analog_level = float(msg_in.data)

    # MaxBotix LV-EZ02:
    # Distance in Inches = Volts / (Vcc/512).
    # For A0, Vcc is 3.3 Volts and signals are in 2^10 levels from 0 to Vcc
    # Therefore (Distance in Meters) = 0.0254 (m/in)*(distance in Inches)

    analog_volts = analog_level * (3.3 / 1024)
    distance_meters = analog_volts / (3.3 / 512) * 0.0254

    # Create and publish distance message
    A1_proc = Float32()
    A1_proc.data = distance_meters

    pub_A1_proc.publish(A1_proc)


def cal_and_pub_encoder(msg_enc):
    left_encoder = float(msg_enc.data[0])

    right_encoder = float(msg_enc.data[1])

    wheel_angle_left = (left_encoder / 1440) * 2 * (np.pi)
    distance_left = wheel_angle_left * 0.03
    wheel_angle_right = (right_encoder / 1440) * 2 * (np.pi)
    distance_right = wheel_angle_right * 0.03

    dist_proc = Float32MultiArray()
    dist_proc.data = [distance_left, distance_right]
    pub_enc_proc.publish(dist_proc)


if __name__ == "__main__":
    try:
        sensors_listener()
    except rospy.ROSInterruptException:
        pass
