#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def publisher():
    pub = rospy.Publisher('cmd_pos', Int32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    # Loop to keep running until program ends
    while not rospy.is_shutdown():
        usr_input = input("Please input 6 motor angles: \n")

        input_split_cs = [int(i) for i in usr_input.split(", ")]
        input_split_s = [int(i) for i in usr_input.split(" ")]

        if (not len(input_split_cs) == 6) and (not len(input_split_s) == 6):
            print("Input is not of length 6, Please try again")
            continue

        msg = Int32MultiArray()
        if (not len(input_split_cs) == 6)
        msg.data = input_split
        
        pub.publish(msg)
        print("Message Send: \n",  msg)
        
        # Wait for message to be sent
        r.sleep()
            
if __name__ == '__main__':
    rospy.init_node('position_publisher', anonymous=True)

    try:
        publisher()
    except rospy.ROSInterruptException: pass