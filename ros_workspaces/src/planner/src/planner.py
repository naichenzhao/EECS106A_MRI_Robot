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
PATH_RAD = 0.13

# center point for path circle
ORIGIN = np.array([0.88, 0.3007, 0.23])

# (aproximate) bounding box for reachable locations
UPPER_BOUNDS = np.array([1.019, 0.2167, 0.2452])
LOWER_BOUNDS = np.array([0.7196, 0.3847, 0.3802])
REF_SPHERE = Sphere(ORIGIN, PATH_RAD)

PRINT_PATH = False

posePub = None
jointPub = None


HOME = [0.05, 0.0845, 0, 0, 0, 0]
REF_LEFT = [0.05, 0, 0, 0, 0, 0]
REF_RIGHT = [0.05, 0.169, 0, 0, 0, 0]

# Listener callback
def callback(message):
    print("recieved message...")
    # Calculate target
    target = message.data
    
    path = generate_path(target)
    
    traj = trajectory_builder(path)
    jointPub.publish(traj)
    print("Completed Path Generation")
    
    
# |-------------------------------------------------------------|
# |      Generate a path of joints to go between
# |-------------------------------------------------------------|    
def generate_path(target):
    pos = target[0:3]
    vec = target[3:6]
    
    circ_pnt = get_intercept(pos, vec)
    
    center = convert_poses(pos, vec)
    center_ik = solve_pose(center)
    
    surface = convert_poses(circ_pnt, vec)
    surface_ik = solve_pose(surface)
    
    # Show poses in Rviz
    disp_poses(center, surface)
    
    # Construct path
    if pos[1] > ORIGIN[1]:
        path = [HOME, REF_LEFT, surface_ik, center_ik, surface_ik, REF_LEFT, HOME]
    else:
        path = [HOME, REF_RIGHT, surface_ik, center_ik, surface_ik, REF_RIGHT, HOME]
        
    return path
    

# |-------------------------------------------------------------|
# |      Use array if joint angles to create a trajectory msg
# |-------------------------------------------------------------|    
def trajectory_builder(path):
    traj = RobotTrajectory()
    traj.joint_trajectory.header.stamp = rospy.Time.now()
    traj.joint_trajectory.header.frame_id = "world"
    traj.joint_trajectory.joint_names = ["x_Gantry", "Y_Gantry", "Z_Gantry", "R_Arm", "TMS_1", "TMS_HEAD"]
    
    for curr_pos in path:
        curr_point = JointTrajectoryPoint()
        curr_point.positions = curr_pos
        
        traj.joint_trajectory.points.append(curr_point)

    print(traj)
    return traj
    
  
  
  
# Uses the IK solver to get the IK result of a specificed pose
def solve_pose(target):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "TMS_gantry"
    # request.ik_request.avoid_collisions = True
    request.ik_request.pose_stamped.header.frame_id = "world"
    request.ik_request.ik_link_name = "TMS_HEAD_Link"
    request.ik_request.pose_stamped.pose = target
    
    response = compute_ik(request)
        
    if response.error_code.val < 0:
        print("[NOTICE:] Point dropped")
        print("Target:", target)
            
    joint_values = response.solution.joint_state.position
    return joint_values

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