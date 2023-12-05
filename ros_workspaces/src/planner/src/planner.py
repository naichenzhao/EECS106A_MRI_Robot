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
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from planner_utils import *

# radius for path circle
PATH_RAD = 0.15

# center point for path circle
ORIGIN = np.array([0.88, 0.3007, 0.23])

# (aproximate) bounding box for reachable locations
UPPER_BOUNDS = np.array([1.019, 0.2167, 0.2452])
LOWER_BOUNDS = np.array([0.7196, 0.3847, 0.3802])
REF_SPHERE = Sphere(ORIGIN, PATH_RAD)

PRINT_PATH = True
LIMIT = 8

posePub = None
jointPub = None

# Listener callback
def callback(message):
    print("recieved message...")
    # Calculate target
    target = message.data
    print("Planning path...")
    path = path_plan(target)
    
    print("Generating trajectory...")
    generate_trajectory(path)
    
    

    
    
    

# |-------------------------------------------------------------|
#   We check if the path is viable and generate a trajectory
# |-------------------------------------------------------------|
def generate_trajectory(path):
    
    # Run Ik solutions on the points
    ik_solutions = []
    for curr in path: 
        ik_point = solve_pose(curr)
        if ik_point != None:
            ik_solutions.append(ik_point)
    
    if len(path) - len(ik_solutions) > LIMIT:
        raise Exception("Number of dropped points less than limit")
    
    trajectory = trajectory_builder(ik_solutions)
    jointPub.publish(trajectory)
    print("Completed Path Generation")
    
    
    
    
            

def solve_pose(target):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "TMS_gantry"
    # request.ik_request.avoid_collisions = True
    request.ik_request.pose_stamped.header.frame_id = "world"
    request.ik_request.ik_link_name = "TMS_HEAD_Link"
    request.ik_request.pose_stamped.pose = target
    
    fail_count = 0
    # Send the request to the service
    while True:
        response = compute_ik(request)
        
        if fail_count > 10:
            print("[NOTICE:] Point dropped")
            print("Target:", target)
            return None
        
        # We will give it 10 retries to make sure it works. Or else we fail it
        if response.error_code.val < 0:
            fail_count += 1
            continue
        else:
            break 
    # print(response)
    joint_values = response.solution.joint_state.position
    return joint_values

    
def trajectory_builder(joint_angles):
    traj = RobotTrajectory()
    traj.joint_trajectory.header.stamp = rospy.Time.now()
    traj.joint_trajectory.header.frame_id = "world"
    traj.joint_trajectory.joint_names = ["x_Gantry", "Y_Gantry", "Z_Gantry", "R_Arm", "TMS_1", "TMS_HEAD"]
    
    for curr_pos in joint_angles:
        curr_point = JointTrajectoryPoint()
        curr_point.positions = curr_pos
        
        traj.joint_trajectory.points.append(curr_point)

    return traj
    
    

# |-------------------------------------------------------------|
#   For path plan, we plan out the points we want to go through
# |-------------------------------------------------------------|
def path_plan(target):
    target_pnt = target[0:3]
    target_vec = target[3:6]
    
    try:
        # Calculate current position
        curr_transform = tfBuffer.lookup_transform("base_link", "TMS_HEAD_Link", rospy.Time(), rospy.Duration(0.1))
        print("found a tf value")
        curr_pnt, curr_vec = transform_to_vec(curr_transform)
    except:
        print("cant find transform, using home")
        curr_pnt = LOWER_BOUNDS
        curr_vec = [0, -1, 0]
        
    
    points, vectors = calclate_points(target_pnt, target_vec, curr_pnt, curr_vec)
    joined = np.vstack((points.T, vectors.T)).T
    
    if PRINT_PATH:
        print("Displaying planned path. Close the window to confirm")
        print_path(joined)
    
    # Calculate poses of the points
    path = convert_poses(joined)
    
    # Publish calculated path to rviz
    msg = PoseArray()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.poses = path[1:]
    posePub.publish(msg) 
        
    return path
    

def calclate_points(target_pnt, target_vec, curr_pnt, curr_vec):
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
        
    target_inter = pick_point(target_a, target_b, target_pnt)
    curr_inter = pick_point(curr_a, curr_b, curr_pnt)
    
    # create path
    len1, vec1 = get_path_linear(curr_inter, curr_pnt, res = 2)
    len2, vec2 = get_path_circular(curr_inter, target_inter, origin = ORIGIN, res = 2)
    len3, vec3 = get_path_linear(target_inter, target_pnt, res = 2)

    if vec1[0][0] > 0:
        vec1 = -1 * np.array(vec1)
    
    if vec3[0][0] > 0:
        vec3 = -1 * np.array(vec3)
    
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

def print_path(joined):
    # create figure
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection ="3d") 
    
    points = joined[:, 0:3]
    vectors = joined[:, 3:6]
    
    # create reference shapes
    plot_sphere(ax, PATH_RAD, center = ORIGIN)

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
    global posePub
    global jointPub
    
    jointPub = rospy.Publisher('TMS/trajectory', RobotTrajectory, queue_size=10) 
    posePub = rospy.Publisher('TMS/path', PoseArray, queue_size=10)
    sphere_pub = rospy.Publisher('TMS/sphere_marker', Marker, queue_size=10)
    rospy.Subscriber("TMS/head_target", Float32MultiArray, callback)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        sphere_pub.publish(gen_sphere())
      

        
if __name__ == '__main__':
    print("path_planner starting up...")
    rospy.wait_for_service('compute_ik')
    rospy.init_node('Path_Planner')
    
    tfBuffer = tf2_ros.Buffer()## initialize a buffer
    tfListener = tf2_ros.TransformListener(tfBuffer)## initialize a tf listener
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    print("planner ready")
    listener()