#!/usr/bin/env python
import rospy
import numpy as np

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray

def publisher():
    pub = rospy.Publisher('head_target', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)
    
    print("== STARTING TARGET_PUBLISHER == ")
    while not rospy.is_shutdown():
        usr_targets(pub)
        r.sleep()


def usr_targets(pub):
    usr_pos = input("Please insert the position:   ")
    usr_vec = input("Please insert the tangent vector:   ")
    
    try:
        pos = np.array([float(i) for i in usr_pos.split(", ")])
        vec = np.array([float(i) for i in usr_vec.split(", ")])
    except:
        print("invalid input. continuing...")
    
    msg = Float32MultiArray()
    print(pos)
    print(vec)
    msg.data = np.concatenate((pos, vec)).tolist()
    
    print("Message Send:",  msg, "\n")
    pub.publish(msg) 


if __name__ == '__main__':
    print("target_publisher starting up...")
    rospy.init_node('target_publisher', anonymous=True)

    try:
        publisher()
    except rospy.ROSInterruptException: pass