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

stl_dimensions = [0.08024935, 0.10076117, 0.11059876] #in meters
#Minimum values for each column: [-34.263749225768464, -51.01442793534265, 0.012813402258831522]
#Maximum values for each column: [42.177958349799546, 48.23456001281738, 110.36154221754808]
scaling_factor = .23/110.34872881528925
global_points = np.array([])
global_normals = np.array([])
clicked = False
pub = rospy.Publisher('TMS/head_target', Float32MultiArray, queue_size=10)
#4x4 homogenous matrix, from world frame to body frame
homogenous_matrix = np.array([[0.0, 0.0, -1, 0.97],
                              [1, 0.0, 0.0, 0.3007],
                              [0.0, -1, 0.0, 0.23],
                              [0.0, 0.0, 0.0, 1.0]])


class InteractivePlot(QMainWindow):
    def __init__(self, points, normals):
        super().__init__()
        self.points = points
        self.normals = normals
        self.initUI()

    def initUI(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # Create a matplotlib figure
        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.scatter(self.points[:, 0],
                        self.points[:, 1], self.points[:, 2])

        # Connect the mouse click event
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.show()
        #publisher()

    def on_click(self, event):
        global global_points
        global global_normals
        if event.inaxes != self.ax:
            return

        # Project 3D data space to 2D data space
        x2, y2, _ = proj3d.proj_transform(
            self.points[:, 0], self.points[:, 1], self.points[:, 2], self.ax.get_proj())

        # Convert 2D data space to 2D screen space
        x2, y2 = np.array([self.ax.transData.transform((x, y))
                          for x, y in zip(x2, y2)]).T

        # Find the closest point
        distance = np.sqrt((x2 - event.x)**2 + (y2 - event.y)**2)
        closest_point_index = np.argmin(distance)
        closest_point = self.points[closest_point_index]
        closest_normal = self.normals[closest_point_index]

        # Clear the previous points and plot again
        self.ax.clear()
        self.ax.scatter(
            self.points[:, 0], self.points[:, 1], self.points[:, 2], s=1, c='blue')

        # Highlight the selected point in red
        self.ax.scatter(
            closest_point[0], closest_point[1], closest_point[2], s=10, c='red')

        # Re-draw the plot
        self.canvas.draw()
        global_points = closest_point
        global_normals = closest_normal
        if not rospy.is_shutdown():
            usr_input(pub)
        #print("clicked")
        #print(f"Closest point: {closest_point}, Normal: {closest_normal}")
    


def usr_input(pub):
    # TODO: read the target from the gui
    # usr_pos = input("Please insert the position:   ")
    

    #vals = np.array([float(i) for i in usr_input.split(", ")])
    try:
    #cast to float
        pos = (global_points * scaling_factor)
        vec = (global_normals)

        #transform to world frame
        pos = np.matmul(homogenous_matrix, np.append(pos, 1))

        vec = np.matmul(homogenous_matrix, np.append(vec, 0))
        
        print(pos)
        print(vec)

        msg = Float32MultiArray()
        msg.data = np.concatenate((pos, vec))
        # # Set pose position
        # msg.position.x = pos[0]
        # msg.position.y = pos[1]
        # msg.position.z = pos[2]
        # # set pose orientation
        # msg.orientation.x = vec[0]
        # msg.orientation.y = vec[1]
        # msg.orientation.z = vec[2]

        print("Message Send:",  msg, "\n")
        pub.publish(msg)
    except:
        print("invalid input. continuing...")


if __name__ == '__main__':
    print("head_publisher starting up...")
    rospy.init_node('head_publisher', anonymous=True)
    # rospy.sleep(3) #wait for 3 seconds

    try:
        # InteractivePlot()
        print("============ STARTING TEST ============ ")
        app = QApplication(sys.argv)
        # load from python binary file
        path = os.getcwd()
        saved_reduced_points = np.load(path+'/src/head_gui/src/points.npy')
        saved_normals = np.load(path+'/src/head_gui/src/normals.npy')
        ex = InteractivePlot(saved_reduced_points, saved_normals)
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
