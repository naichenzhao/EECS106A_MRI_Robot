#!/usr/bin/env python
import tf2_ros
import rospy
import sys
import time



def something(target_frame, source_frame):
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("nec", "robo", rospy.Time(), rospy.Duration(0.1))
            #no need for listener
            print(trans)
            time.sleep(0.1)
            
        except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
            print("FUCK")



if __name__ == '__main__':
    #splitting arguments from terminal
    target = "base_link"
    source = "TMS_HEAD_LINK"
    rospy.init_node('listener', anonymous=True)
    something(target, source)