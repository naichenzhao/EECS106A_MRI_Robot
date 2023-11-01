#!/usr/bin/env python
import serial 

import rospy
from std_msgs.msg import Int32MultiArray


def callback(message):
    # Get angles
    angles = message.data
    print(": I heard:", angles)

    # Send to microcontroller
    str_send = str(angles)[1:-1] + "\n"
    STM.write(str_send.encode("utf-8"))


# Define the method which contains the node's main functionality
def listener():
    rospy.Subscriber("cmd_pos", Int32MultiArray, callback)

    # Wait till next message
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    STM = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

    listener()