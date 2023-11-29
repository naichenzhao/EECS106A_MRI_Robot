#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *

def publisher():
    pub = rospy.Publisher('gantry_pose', Pose, queue_size=10)
    
    pos = [0.705, -0.173, 0.470]
    quat = [0.218, -0.0658, 0.9566, 0.182]
    msg = gen_pose(pos, quat)
    
    # pub.publish(msg)
    
    print("============ STARTING TEST ============ ")
    test_path(pub)
    

def test_path(pub):  
    
    pos0 = [0.705, -0.173, 0.470]
    pos1 = [0.98, -0.32, 0.35]
    quat = [0.218, -0.0658, 0.9566, 0.182]
    while not rospy.is_shutdown():
        input("Press [ENTER] to go next")
        print("    ====== GOING TO LOCATION")
        msg = gen_pose(pos1, quat)
        pub.publish(msg)
        rospy.sleep(10)
        input("Press [ENTER] to go next")
    
        print("    ====== GOING TO HOME")
        msg = gen_pose(pos0, quat)
        pub.publish(msg)
        rospy.sleep(10)
    
    

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