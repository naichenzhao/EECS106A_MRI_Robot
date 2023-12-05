#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import numpy as np
import scipy
import matplotlib.pylab as plt
import math
from skspatial.objects import Line, Sphere
from skspatial.plotting import plot_3d
from tf2_geometry_msgs import do_transform_pose

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from planner_utils import *

# radius for path circle
PATH_RAD = 0.12

# center point for path circle
ORIGIN = np.array([0.89, 0.3007, 0.24])

# (aproximate) bounding box for reachable locations
UPPER_BOUNDS = np.array([1.019, 0.2167, 0.2452])
LOWER_BOUNDS = np.array([0.7196, 0.3847, 0.3802])
REF_SPHERE = Sphere(ORIGIN, PATH_RAD)

pub = None

# Listener callback
def callback(message):
    print("recieved message...")
    # Calculate target
    target = message.data
    target_pnt = target[0:3]
    target_vec = target[3:6]
    
    
    try:
        # Calculate current position
        curr_transform = tfBuffer.lookup_transform("world", "TMS_HEAD_Link", rospy.Time(), rospy.Duration(0.1))
        # print(curr_transform)
        curr_pnt, curr_vec = transform_to_vec(curr_transform)
    except:
        print("cant find transform, using home")
        curr_pnt = LOWER_BOUNDS
        curr_vec = [0, -1, 0]
    
    points, vectors = calclate_path(target_pnt, target_vec, curr_pnt, curr_vec)
    joined = np.vstack((points.T, vectors.T)).T
    # print("joined:\n", joined)
    
    print("Displaying planned path")
    print_path(points, vectors)
    
    path = convert_poses(joined)
    msg = PoseArray()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.poses = path[1:]
    print("publishing pose")
    pub.publish(msg) 
    
    

def calclate_path(target_pnt, target_vec, curr_pnt, curr_vec):
    target_line = Line(target_pnt, target_vec)
    curr_line = Line(curr_pnt, curr_vec)
    
    #Calculate intersection Points
    target_a, target_b = REF_SPHERE.intersect_line(target_line)
    try:
        curr_a, curr_b = REF_SPHERE.intersect_line(curr_line)
    except:
        print("No intersection found")
        print("current point is:", curr_pnt)
        
        bounded_pnt = bound_point(curr_pnt)
        new_line = Line(bounded_pnt, (curr_pnt - ORIGIN))
        curr_a, curr_b = REF_SPHERE.intersect_line(new_line)
        
    target_inter = pick_point(target_a, target_b)
    curr_inter = pick_point(curr_a, curr_b)
    
    # create path
    len1, vec1 = get_path_linear(curr_pnt, curr_inter, res = 4)
    len2, vec2 = get_path_circular(curr_inter, target_inter, origin = ORIGIN, res = 10)
    len3, vec3 = get_path_linear(target_inter, target_pnt, res = 4)
    
    points = np.concatenate((len1, len2, len3))
    vectors = np.concatenate((vec1, vec2, vec3))
    
    return points, vectors

def convert_poses(vecs):
    poses = []
    quat = [0, 1, 0, 0]
    for v in vecs:
        quat = vect_to_quat(v[3:6].T)
        p = Pose()
        p.position.x = v[0]
        p.position.y = v[1]
        p.position.z = v[2]
        
        p.orientation.x = quat[0]
        p.orientation.y = quat[1]
        p.orientation.z = quat[2]
        p.orientation.w = quat[3]
        poses.append(p) 
    return poses

def print_path(points, vectors):
    # create figure
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection ="3d") 
    
    # create reference shapes
    plot_sphere(ax, PATH_RAD, center = ORIGIN)
    plot_cube(ax, LOWER_BOUNDS, UPPER_BOUNDS)

    # Plot path
    ax.scatter3D(points[:, 0], points[:, 1], points[:, 2], color = "green" )
    ax.plot(points[:, 0], points[:, 1], points[:, 2], color = "green" )
    for i in range(len(points)):
        plot_vec(ax, points[i], vectors[i], colour = "orange" )


    ax.set_xlim3d([0.7, 1.05])
    ax.set_ylim3d([0.15, 0.45])
    ax.set_zlim3d([0.2, 0.5])
    
    ax.set_xlabel('X', fontsize=20)
    ax.set_ylabel('Y', fontsize=20)
    ax.set_zlabel('Z', fontsize=20)
    plt.show()
       
def bound_point(pnt):
    new_x = max(min(pnt[0], UPPER_BOUNDS[0]), LOWER_BOUNDS[0])
    new_y = max(min(pnt[1], UPPER_BOUNDS[1]), LOWER_BOUNDS[1])
    new_z = max(min(pnt[2], UPPER_BOUNDS[2]), LOWER_BOUNDS[2])
    
    return np.array([new_x, new_y, new_z])

def listener():
    global pub
    pub = rospy.Publisher('TMS_path', PoseArray, queue_size=10)
    sphere_pub = rospy.Publisher('sphere_marker', Marker, queue_size=10)
    bounds_pub = rospy.Publisher('bounds_marker', Marker, queue_size=10)
    rospy.Subscriber("head_target", Float32MultiArray, callback)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        sphere_pub.publish(gen_sphere())
      
def gen_sphere():
    m = Marker() 
    m.type = 2  
    m.header.frame_id = "world"
        
    m.color.r = 255
    m.color.g = 255
    m.color.b = 255
    m.color.a = 0.5
        
    m.pose.position.x = ORIGIN[0]
    m.pose.position.y = ORIGIN[1]
    m.pose.position.z = ORIGIN[2]
        
    m.pose.orientation.x = 0
    m.pose.orientation.y = 1
    m.pose.orientation.z = 0
    m.pose.orientation.w = 0
        
    m.scale.x =PATH_RAD * 2
    m.scale.y =PATH_RAD * 2
    m.scale.z =PATH_RAD * 2
    
    return m
        
if __name__ == '__main__':
    print("target_publisher starting up...")
    rospy.init_node('target_publisher', anonymous=True)
    
    tfBuffer = tf2_ros.Buffer()## initialize a buffer
    tfListener = tf2_ros.TransformListener(tfBuffer)## initialize a tf listener
    print("planner ready")
    listener()