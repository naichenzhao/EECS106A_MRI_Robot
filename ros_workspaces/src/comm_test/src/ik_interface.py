#!/usr/bin/env python
import serial 
import rospy
import numpy as np

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander


# Define global variables

# Origins and limits from URDF file
limits = np.array([-0.3, 0.16, -0.137, -3.14, -3.14, -3.14])
refs = np.array([0, 0, 0, 0, 0, 0])
pub = None

idle = True
simulated = False

# Listener callback
def callback(message):
    # Construct the request
    global idle
    idle = False
    rospy.sleep(0.5)
    
    request = GetPositionIKRequest()
    request.ik_request.avoid_collisions = True
    request.ik_request.group_name = "TMS_gantry"
    request.ik_request.pose_stamped.header.frame_id = "base_link"
    request.ik_request.ik_link_name = "TMS_HEAD_Link"
    request.ik_request.pose_stamped.pose = message
    

    input('Press [ Enter ] to begin')
    while not rospy.is_shutdown():
        try:
            # Send the request to the service
            # print(request)
            response = compute_ik(request)
            
            # print(response)
            joint_values = response.solution.joint_state.position
            print("\nresponse:", joint_values)
            
            group = MoveGroupCommander("TMS_gantry")
            group.set_pose_target(request.ik_request.pose_stamped)
            
            if response.error_code.val < 0:
                print("IK Solve failed. Exiting Solver\n")
                break
            
            user_input = input("Enter 'y' if the trajectory looks safe, 'n' to cancel: ")

            # If movement looks viable
            if user_input == 'y':
                raw_values = np.array(response.solution.joint_state.position)
                print("raw values: ", raw_values)
                if simulated:
                    push_states(joint_values)
                else:
                    scaled_values = convert_values(raw_values)
                    send_data(scaled_values)
                print("Completed Movement\n")
                break
            elif user_input == 'n':
                print("Cancelled Movement\n")
                break
        except rospy.ServiceException as e:
            print("SKILL ISSUE: %s"%e)
    idle = True
        
        
def listener():
    global pub
    rospy.Subscriber("gantry_pose", Pose, callback)
    
    
    while not rospy.is_shutdown():
        # Read Data
        if not simulated and STM.isOpen() and idle:
            read_val = str(STM.readline().decode('utf-8'))
            if read_val != "" and read_val[0] == 'd':
                pr_data = str(read_val[1:-3])
                # print(pr_data.split(" "))
                try:
                    encoder_angles = np.array([int(i) for i in pr_data.split(" ")])
                    push_states(deconv_values(encoder_angles))
                except:
                    print("failed split")


def push_states(angles):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ["x_Gantry", "Y_Gantry", "Z_Gantry", "R_Arm", "TMS_1", "TMS_HEAD"]
    js.position = angles
    # print(angles)
    pub.publish(js)
    
    
def convert_values(joint_values):  
    angles = np.around(10000 * (joint_values + refs)/limits, 0)
    joints = [angles[0], angles[1], angles[2], angles[3], 0, 0]
    joints[4] = int((-angles[4] + angles[5])/2) % 10000
    joints[5] = int((angles[4] + angles[5])/2) % 10000
    return joints

def deconv_values(encoder_angles):  
    joints = ((encoder_angles/10000) * limits) - refs
    angles = [joints[0], joints[1], joints[2], joints[3], 0, 0]
    angles[4] = (-joints[4] + joints[5])
    angles[5] = (joints[4] + joints[5])
    return angles
    
    
def send_data(joints):
    print("Sending Positions:", joints)

    # Send to microcontroller
    str_send = "p" + str(joints)[1:-1] + "\n"
    STM.write(str_send.encode("utf-8"))

# Python's syntax for a main() method
if __name__ == '__main__':    
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('ik_service_query')
    
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
        
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    listener()