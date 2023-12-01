#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *

def publisher():
    pub = rospy.Publisher('gantry_pose', Pose, queue_size=10)
    
    print("============ STARTING TEST ============ ")
    while not rospy.is_shutdown():
        usr_input(pub)
    

# Go to two ends of the reachable workspace
def test_path(pub):  
    
    pos = [0.7196, 0.3847, 0.3802]
    pos2 = [1.014, 0.2167, 0.2452]
    quat = [0, 1, 0, 0]
    
    input("Press [ENTER] to go next")
    print("    ====== GOING TO LOCATION")
    msg = gen_pose(pos, quat)
    pub.publish(msg)

def usr_input(pub):
    print("Note tha approximate bounds of the workspace:")
    print("     max: [1.019, 0.2167, 0.2452]")
    print("     min: [0.7196, 0.3847, 0.3802]")
    usr_pos = input("Please insert the position:   ")
    usr_quat = input("Please insert the orientation. Leave blank for default:   ")
    
    pos = [float(i) for i in usr_pos.split(", ")]
    if len(usr_quat) < 3:
        quat = [0, 1, 0, 0]
    else:
        quat = [float(i) for i in usr_quat.split(", ")]
    print(pos, quat)
    msg = gen_pose(pos, quat)
    pub.publish(msg)
    

# Generate a pose from coordinates. Mainly here to make code cleaner
def gen_pose(pos, quat):
    p = Pose()
    # Set pose position
    p.position.x = pos[0]
    p.position.y = pos[1]
    p.position.z = pos[2]
    # set pose orientation
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return p


     
if __name__ == '__main__':
    print("pose_publisher starting up...")
    rospy.init_node('pose_publisher', anonymous=True)
    # rospy.sleep(3) #wait for 3 seconds 

    try:
        publisher()
    except rospy.ROSInterruptException: pass