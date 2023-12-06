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

pub = rospy.Publisher('TMS/head_pointcloud', PointCloud, queue_size=10)
path = os.getcwd()


def transform_points():
    # cast to float
    saved_reduced_points = np.load(path+'/src/head_gui/src/points.npy')
    saved_normals = np.load(path+'/src/head_gui/src/normals.npy')

    saved_reduced_points = (saved_reduced_points * scaling_factor)

    # transform to world frame
    for i in range(len(saved_reduced_points)):
        saved_reduced_points[i] = np.matmul(
            homogenous_matrix, np.append(saved_reduced_points[i], 1))[0:3]
    for i in range(len(saved_normals)):
        saved_normals[i] = np.matmul(
            homogenous_matrix, np.append(saved_normals[i], 0))[0:3]

    msg = PointCloud()

    msg.points = [Point32(x=saved_reduced_points[i][0], y=saved_reduced_points[i]
                          [1], z=saved_reduced_points[i][2]) for i in range(len(saved_reduced_points))]
    msg.header.frame_id = "world"

    pub.publish(msg)


if __name__ == '__main__':
    print("pointcloud_publisher starting up...")
    rospy.init_node('head_publisher', anonymous=True)
    # rospy.sleep(3) #wait for 3 seconds

    try:
        # InteractivePlot()
        print("============ STARTING TEST ============ ")
        while not rospy.is_shutdown():
            transform_points()

    except rospy.ROSInterruptException:
        pass
