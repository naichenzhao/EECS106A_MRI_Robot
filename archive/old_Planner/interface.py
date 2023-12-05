#!/usr/bin/env python
import serial 
import rospy
import numpy as np
import time

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from moveit_commander import MoveGroupCommander


# Define global variables

# Origins and limits from URDF file
limits = np.array([-0.3, 0.169, -0.137, -3.14, -3.14, -3.14])

scale = np.array([10, 20, 20, 1, 1, 1])
pub = None

idle = True
simulated = False

LIMIT = 1000

# Listener callback
def callback(message):
    global idle
    idle = False

    usrin = input('Press [ Enter ] to begin, [n] to exit]')
    if usrin == 'n':
        return
    print("Executing Path")
    
    operate_path(message)
    print("Finished executing Path")
    idle = True

          
def operate_path(trajectory):
    waypoints = trajectory.joint_trajectory.points
    for current_target in waypoints:
        movegroup(current_target.positions)
        
    
# Move robot to a certain point
def movegroup(current_target):
    send_pose(current_target)
    
    if simulated :
        time.sleep(1)
    
    while not simulated:
        send_pose(current_target)
        read_val = str(STM.readline().decode('utf-8'))
        encoder_angles = parse_input(read_val)
            
        if len(encoder_angles) == 6:
            push_states(deconv_values(encoder_angles))
            
            refval = np.linalg.norm(convert_values(np.array(current_target)) - np.array(encoder_angles) )
            if refval < LIMIT:
                print("Reached Waypoint!")
                break
            else:
                print(refval)


def send_pose(joint_values):
    raw_values = np.array(joint_values)
    if simulated:
        push_states(joint_values)
    else:
        scaled_values = convert_values(raw_values)
        send_data(scaled_values)

    
def listener():
    global pub
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber("TMS/trajectory", RobotTrajectory, callback)
    while not rospy.is_shutdown():
        # Read Data
        if not simulated and STM.isOpen() and idle:
            read_val = str(STM.readline().decode('utf-8'))
            joint_angles = parse_input(read_val)
            
            if len(joint_angles) == 6:
                # print(joint_angles)
                push_states(deconv_values(joint_angles))
            

def parse_input(read_val):
    if read_val != "" and read_val[0] == 'd':
        pr_data = str(read_val[1:-3])
        try:
            angles = [int(i) for i in pr_data.split(" ")]
            return angles
        except:
            print("failed split")
    return []

def push_states(angles):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ["x_Gantry", "Y_Gantry", "Z_Gantry", "R_Arm", "TMS_1", "TMS_HEAD"]
    js.position = angles
    pub.publish(js)
    
    
def convert_values(joint_values):  
    angles = np.clip(np.around(10000 * joint_values/limits), 0, 10000)
    joints = np.array([angles[0], angles[1], angles[2], angles[3], 0, 0])
    joints[4] = int((-angles[4] + angles[5])/2)
    joints[5] = int((angles[4] + angles[5])/2)
    return joints


def deconv_values(encoder_input):
    encoder_angles = np.array(encoder_input)  
    joints = (encoder_angles/10000) * limits
    angles = [joints[0], joints[1], joints[2], joints[3], 0, 0]
    angles[4] = (-joints[4] + joints[5])
    angles[5] = (joints[4] + joints[5])
    return np.array(angles)
    
    
def send_data(joints):
    print("Sending Positions:", joints)
    # Send to microcontroller
    str_send = "p" + str(joints)[1:-1] + "\n"
    STM.write(str_send.encode("utf-8"))





if __name__ == '__main__':
    rospy.init_node('STM32_Interface') 
    
    STM = None
    # Test different ports until one sticks. If not, rais an error
    for i in range(10):
        try:
            STM = serial.Serial(port='/dev/ttyACM' + str(i), baudrate=115200, timeout=1)
            print("connecting to STM32 on port: /dev/ttyACM"+ str(i))
            break
        except:
            print("error connecting to port /dev/ttyACM" + str(i))
            print("     retrying on a different port...")        
    
    if STM == None:
        print("\nAssuming we are simulating the arm")
        simulated = True
        # raise Exception("Unable to detect STM32 controller")
    
    listener()