#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped    
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('/tms/force', Float32MultiArray, queue_size=10)
def listener():
    rospy.Subscriber("/wireless_ft/wrench_1", WrenchStamped, callback)
    rospy.spin()
def callback(message):
    # Calibration matrix
    M = np.array([[-7.29630143e-04,  7.11612252e-04, -8.31278271e-04, 1.66688749e-03, -5.92233414e-04, 3.54268295e-04,-4.60483989e+00, -4.60483989e+00, -4.60483989e+00, -4.60483989e+00, -4.60483989e+00, -4.60483989e+00], [ 3.17993720e-06,  1.89501607e-05,  6.96016742e-06, -2.05404155e-05, -8.63993263e-06, -3.25830594e-06, -5.29858052e-02, -5.29858052e-02, -5.29858052e-02, -5.29858052e-02, -5.29858052e-02, -5.29858052e-02], [-7.09921133e-06, -1.42473405e-05,  6.32440623e-06, 2.78427870e-05,  4.55780419e-07, -4.51283913e-06, -2.97625580e-02, -2.97625580e-02, -2.97625580e-02, -2.97625580e-02, -2.97625580e-02, -2.97625580e-02]])
    x = np.array([message.wrench.force.x, message.wrench.force.y, message.wrench.force.z,
                  message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z,
                  1,1,1,1,1,1])
    F = np.matmul(M,x)
    msg = Float32MultiArray()
    msg.data = F
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('ft_transform', anonymous=True)
    try:
        listener()
    except rospy.ROSInterruptException: pass