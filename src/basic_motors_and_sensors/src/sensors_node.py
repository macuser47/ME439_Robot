#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Node to a continually read data from a Serial port ('/dev/ttyUSB0' (Arduino
Nano on USB) and publish it to relevant topics 
"""


import rospy
import serial
import traceback
from std_msgs.msg import Int32


# Publish sensors data at the rate it comes in
# Here we publish:
#    Analog value of interest (levels)
#
# Here we publish each in a separate topic.
# In the long-term it would be better to publish them together.


def sensors_reader():
    rospy.init_node("sensors_node", anonymous=False)

    # Create a publisher for each sensor
    pub_A0 = rospy.Publisher("/light_dark", Int32, queue_size=1)
    pub_A1 = rospy.Publisher("/ultrasonic", Int32, queue_size=1)

    msg_A0 = Int32()
    msg_A1 = Int32()

    # ----------setup serial--------------
    ser = serial.Serial(
        "/dev/ttyUSB0"
    )  # serial port to alamode is /dev/ttyS0. # port to Arduino Nano is /dev/ttyUSB0
    ser.baudrate = 57600
    ser.bytesize = 8
    ser.parity = "N"
    ser.stopbits = 1
    ser.timeout = 1  # one second time out.

    ser.flush()  # Flush any data currently on the port
    ser.readline()

    while not rospy.is_shutdown():

        try:
            # Read the serial port for a string that looks like "e0:123456",
            # with format [INPUT NAME]:[VALUE]
            line = ser.readline().decode().strip()
            line = line.split(":")
            data_type = line[0]
            data_value = int(line[1])

            if data_type == "A0":
                msg_A0 = data_value  # Analog reading
                pub_A0.publish(msg_A0)
            elif data_type == "A1":
                msg_A1 = data_value  # Analog reading
                pub_A1.publish(msg_A1)
            else:
                continue

        except Exception:
            traceback.print_exc()
            pass


if __name__ == "__main__":
    try:
        sensors_reader()
    except rospy.ROSInterruptException:
        pass
