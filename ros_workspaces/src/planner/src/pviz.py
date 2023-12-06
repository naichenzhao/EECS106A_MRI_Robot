#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import numpy as np
import scipy
import matplotlib.pylab as plt

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from planner_utils import gen_sphere

# radius for path circle
PATH_RAD = 0.13
HEAD_RAD = (0.13/2)
# center point for path circle
ORIGIN = np.array([0.84, 0.3007, 0.23])
HEAD_ORIGIN = np.array([0.84, 0.3007, 0.23])

# (aproximate) bounding box for reachable locations
UPPER_BOUNDS = np.array([1.019, 0.2167, 0.2452])
LOWER_BOUNDS = np.array([0.7196, 0.3847, 0.3802])

PRINT_PATH = False

posePub = None
jointPub = None


HOME = [-0.05, 0.0845, -0.01, 0, 0, 0]
REF_LEFT = [-0.05, 0, -0.01, 0, 0, 0]
REF_RIGHT = [-0.05, 0.169, -0.01, 0, 0, 0]


# Display poses in Rviz
def disp_poses(center, surface):
    msg = PoseArray()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.poses = [center, surface]
    posePub.publish(msg)
       
    

def listener():
    global posePub
    global jointPub
    
    posePub = rospy.Publisher('TMS/path', PoseArray, queue_size=10)
    sphere_pub = rospy.Publisher('TMS/sphere_marker', Marker, queue_size=10)
    head_pub = rospy.Publisher('TMS/head_marker', Marker, queue_size=10)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        sphere_pub.publish(gen_sphere(PATH_RAD, ORIGIN))
        head_pub.publish(gen_sphere(HEAD_RAD, HEAD_ORIGIN, colour = (0, 255, 255, 1)))
      
        
if __name__ == '__main__':
    print("path_planner starting up...")
    rospy.init_node('Path_Planner')
    
    tfBuffer = tf2_ros.Buffer()## initialize a buffer
    tfListener = tf2_ros.TransformListener(tfBuffer)## initialize a tf listener
    
    print("planner ready")
    listener()