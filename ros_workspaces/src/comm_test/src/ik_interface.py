#!/usr/bin/env python
import serial 
import rospy
import numpy as np

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander


# Define global variables

# Origins and limits from URDF file
origins = np.array([0, 0, 0, 0, 0, 0])
limits = np.array([-0.3, 0.16, -0.137, 6.28, 6.28, 6.28])

idle = True

# Listener callback
def callback(message):
    # Construct the request
    global idle
    idle = False
    request = GetPositionIKRequest()
    request.ik_request.group_name = "TMS_gantry"
    request.ik_request.pose_stamped.pose = message

    input('Press [ Enter ] to begin')
    while not rospy.is_shutdown():
        try:
            # Send the request to the service
            print(request)
            response = compute_ik(request)
            
            # print(response)
            joint_values = response.solution.joint_state.position
            print("\nresponse:", joint_values)
            
            group = MoveGroupCommander("TMS_gantry")
            group.set_pose_target(request.ik_request.pose_stamped)
            
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe, 'n' to cancel: ")

            # If movement looks viable
            if user_input == 'y':
                raw_values = np.array(response.solution.joint_state.position)
                print("raw values: ", raw_values)
                scaled_values = convert_values(raw_values).tolist()
                send_data(scaled_values)
                idle = True
                break
            elif user_input == 'n':
                break
        except rospy.ServiceException as e:
            print("SKILL ISSUE: %s"%e)
        
        
def listener():
    rospy.Subscriber("gantry_pose", Pose, callback)
    
    
    # while not rospy.is_shutdown():
    #     if STM.isOpen() and idle:
    #         read_val = str(STM.readline().decode('utf-8'))
    #         if read_val != "":
    #             pr_data = str(read_val[:-1])
    #             print(pr_data)
    rospy.spin()
    
    
def convert_values(joint_values):  
    return np.around(10000 * (joint_values - origins)/limits, 0)
    
    
def send_data(joints):
    print("Sending Positions:", joints)

    # Send to microcontroller
    str_send = "p" + str(joints)[1:-1] + "\n"
    STM.write(str_send.encode("utf-8"))

# Python's syntax for a main() method
if __name__ == '__main__':    
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
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
        print("SKILL ISSUE")
        # raise Exception("Unable to detect STM32 controller")
        
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    listener()