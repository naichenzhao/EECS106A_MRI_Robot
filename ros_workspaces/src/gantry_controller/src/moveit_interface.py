#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()


    group_name = "TMS_controller"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    
    # while(1):
    #     print_state(move_group, robot)
    #     rospy.sleep(0.5)
    
    while not rospy.is_shutdown():
        print("============ GOING TO HOME")
        go_home(move_group)
        rospy.sleep(7)
    
        print("============ GOING TO OTHER LOCATION")
        go_pos(move_group)
        rospy.sleep(7)
    
    print("============ GOING TO HOME")
    go_home(move_group)
    rospy.sleep(0.5)


def print_state(move_group, robot):
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # print("============ Printing robot state")
    # print(robot.get_current_state())
    # print("")
    
    print("============ Current Pose")
    print(move_group.get_current_pose().pose)
    print("")
        
    rospy.sleep(1.)


def go_home(move_group): 
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.21782031433975177
    pose_goal.orientation.y = -0.06581137766911603
    pose_goal.orientation.z = 0.956569153400718
    pose_goal.orientation.w = 0.1822049066091771
    
    pose_goal.position.x = 0.705
    pose_goal.position.y = -0.173
    pose_goal.position.z = 0.470

    move_group.set_pose_target(pose_goal)

    success = move_group.go(wait=True)
    print("Movement status:" + str(success))
    
    move_group.stop()
    move_group.clear_pose_targets()

def go_pos(move_group): 
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.21782031433975177
    pose_goal.orientation.y = -0.06581137766911603
    pose_goal.orientation.z = 0.956569153400718
    pose_goal.orientation.w = 0.1822049066091771
    
    pose_goal.position.x = 0.98
    pose_goal.position.y = -0.32
    pose_goal.position.z = 0.35

    move_group.set_pose_target(pose_goal)

    success = move_group.go(wait=True)
    print("Movement status:" + str(success))
    
    move_group.stop()
    move_group.clear_pose_targets()


if __name__ == '__main__':

    # Run main at startup
    main()
