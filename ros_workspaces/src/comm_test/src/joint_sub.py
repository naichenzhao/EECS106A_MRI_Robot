#!/usr/bin/env python
import serial 

import rospy
from std_msgs.msg import Int32MultiArray


def callback(message):
    # Get angles
    angles = message.data
    print(": I heard:", angles)

    # Send to microcontroller
    str_send = "p" + str(angles)[1:-1] + "\n"
    STM.write(str_send.encode("utf-8"))


# Define the method which contains the node's main functionality
def listener():
    rospy.Subscriber("joint_pos", Int32MultiArray, callback)
    
    while not rospy.is_shutdown():
        if STM.isOpen():
            read_val = str(STM.readline().decode('utf-8'))
            if read_val != "":
                pr_data = str(read_val[:-1])
                print(pr_data)
    


if __name__ == '__main__':
    rospy.init_node('joint_Interface', anonymous=True)

    STM = None

    # Test different ports until one sticks. If not, rais an error
    for i in range(10):
        try:
            STM = serial.Serial(port='/dev/ttyACM' + str(i), baudrate=115200, timeout=1)
            print("connecting to STM32 on port: /dev/ttyACM"+ str(i))
            break
        except:
            print("error connecting to port /dev/ttyACM" + str(i))
            print("     retrying on a different port...")        
    
    if STM == None:
        raise Exception("Unable to detect STM32 controller")
    
    listener()
    
    # while True:
    #     cc=str(STM.readline())
    #     print(cc)
    