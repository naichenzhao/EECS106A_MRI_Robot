#!/usr/bin/env python
import rospy
import os
from geometry_msgs.msg import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import proj3d
from sensor_msgs.msg import PointCloud
import tf2_ros
from visualization_msgs.msg import Marker
import time

from head_pub import homogenous_matrix

stl_dimensions = [0.08024935, 0.10076117, 0.11059876]  # in meters
# Minimum values for each column: [-34.263749225768464, -51.01442793534265, 0.012813402258831522]
# Maximum values for each column: [42.177958349799546, 48.23456001281738, 110.36154221754808]
scaling_factor = .23/110.34872881528925
global_points = np.array([])
global_normals = np.array([])
clicked = False
# pub = rospy.Publisher('TMS/head_target', Float32MultiArray, queue_size=10)
# 4x4 homogenous matrix, from world frame to body frame

# radius for path circle
PATH_RAD = 0.16
HEAD_RAD = (0.13/2)

pub = rospy.Publisher('TMS/head_pointcloud', PointCloud, queue_size=10)
sphere_pub = rospy.Publisher('TMS/sphere_marker', Marker, queue_size=10)
head_pub = rospy.Publisher('TMS/head_marker', Marker, queue_size=10)
path = os.getcwd()


head_offset = [1.08, 0.283, -0.545]
sphere_offset = [0.91, 0.2915, -0.523]

def transform_points():
    # cast to float
    saved_reduced_points = np.load(path+'/src/head_gui/src/points.npy')
    saved_normals = np.load(path+'/src/head_gui/src/normals.npy')

    saved_reduced_points = (saved_reduced_points * scaling_factor)

    # transform to world frame
    for i in range(len(saved_reduced_points)):
        saved_reduced_points[i] = np.matmul(
            homogenous_matrix, saved_reduced_points[i])
    for i in range(len(saved_normals)):
        saved_normals[i] = np.matmul(
            homogenous_matrix, saved_normals[i])
        
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform("nec", "world", rospy.Time(), rospy.Duration(0.1))
        origin = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        sphere_pub.publish(gen_sphere(PATH_RAD, origin + sphere_offset))
        head_pub.publish(gen_sphere(HEAD_RAD, origin + sphere_offset, colour = (0, 255, 255, 1)))
        pub.publish(gen_head(origin + head_offset, saved_reduced_points))
        
        print(origin)
        rospy.sleep(0.2)
    except:
        return

def gen_sphere(rad, origin, colour = (255, 255, 255, 0.5)):
    m = Marker() 
    m.type = 2  
    m.header.frame_id = "world"
        
    m.color.r = colour[0]
    m.color.g = colour[1]
    m.color.b = colour[2]
    m.color.a = colour[3]
        
    m.pose.position.x = origin[0]
    m.pose.position.y = origin[1]
    m.pose.position.z = origin[2]
        
    m.pose.orientation.x = 0
    m.pose.orientation.y = 1
    m.pose.orientation.z = 0
    m.pose.orientation.w = 0
        
    m.scale.x =rad * 2
    m.scale.y =rad * 2
    m.scale.z =rad * 2
    
    return m    


def gen_head(origin, saved_reduced_points):
    msg = PointCloud()
    msg.points = [Point32(x=(saved_reduced_points[i][0] + origin[0]), y=(saved_reduced_points[i] + origin[1])
                            [1], z=(saved_reduced_points[i][2]+ origin[2])) for i in range(len(saved_reduced_points))]
    msg.header.frame_id = "world"
    return msg
    


if __name__ == '__main__':
    print("pointcloud_publisher starting up...")
    rospy.init_node('head_publisher', anonymous=True)
    # rospy.sleep(3) #wait for 3 seconds

    try:
        print("Publishing Points")
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            transform_points()

    except rospy.ROSInterruptException:
        pass
