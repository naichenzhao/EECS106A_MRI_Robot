#!/usr/bin/env python
import serial 
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "TMS_controller"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "TMS_HEAD_link"

        # request.ik_request.ik_link_name = link
        # request.ik_request.pose_stamped.header.frame_id = "base_link"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.98
        request.ik_request.pose_stamped.pose.position.y = -0.32
        request.ik_request.pose_stamped.pose.position.z = 0.35      
        request.ik_request.pose_stamped.pose.orientation.x = 0.218
        request.ik_request.pose_stamped.pose.orientation.y = -0.0658
        request.ik_request.pose_stamped.pose.orientation.z = 0.9566
        request.ik_request.pose_stamped.pose.orientation.w = 0.182
        
        input('\n\nPress [ Enter ]: \n\n')
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            joint_values = response.solution.joint_state.position
            print(joint_values)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        exit()

# Python's syntax for a main() method
if __name__ == '__main__':
    main()