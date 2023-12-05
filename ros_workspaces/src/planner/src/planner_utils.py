import numpy as np
from planner import PATH_RAD, ORIGIN
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from skspatial.objects import Line, Sphere
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion


def get_intercept(pos, vec):
    target_line = Line(pos, vec)
    ref_sphere = Sphere(ORIGIN, PATH_RAD)

    target_a, target_b = ref_sphere.intersect_line(target_line)
    return pick_point(target_a, target_b)

def pick_point(p1, p2):
    if (p1[0] - ORIGIN[0]) > 0 and abs((p1[1] - ORIGIN[1])) < PATH_RAD and (p1[2] - ORIGIN[2]) > 0:
        return p1
    return p2   

def convert_poses(pos, vec):
    quat = vect_to_quat(vec)
    p = Pose()
    p.position.x = pos[0]
    p.position.y = pos[1]
    p.position.z = pos[2]
        
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    
    return p





def get_path_linear(p1, p2, res = 5):
    points = np.zeros((res+1, 3))
    points[:, 0] = np.linspace(p1[0], p2[0], res+1)
    points[:, 1] = np.linspace(p1[1], p2[1], res+1)
    points[:, 2] = np.linspace(p1[2], p2[2], res+1)
    
    vecs = [(p1 - p2)/ np.linalg.norm(p1 - p2)] * (res+1)
    return points[1:], vecs[1:]



def get_angles(p, center):
    q = np.array(p) - np.array(center)
    r = np.sqrt(q[0]**2 + q[1]**2 + q[2]**2)
    if q[0] == 0:
        theta = (np.sign(q[1]) * np.pi/2) + np.pi
    else:
        theta = (np.arctan(q[1]/q[0])) + np.pi
    if q[2] == 0:
        phi = np.pi/2
    else:
        phi = np.arctan((np.sqrt(q[0]**2 + q[1]**2))/q[2])
    return(r, theta, phi)


def get_rpy(v1): 
    roll = np.pi/2
    pitch = np.arcsin(v1[1]/np.linalg.norm(v1))
    yaw = np.pi/2-np.arctan2(v1[0]/np.linalg.norm(v1), v1[2]/np.linalg.norm(v1)) 
    return roll, pitch, yaw



def vect_to_quat(v):
    roll, pitch, yaw = get_rpy(v)
    # return quaternion_from_euler(0 - np.pi/2, pitch + np.pi/2, yaw + np.pi/2)
    return quaternion_from_euler(yaw, pitch, roll)



def transform_to_vec(tf):
    v2 = np.array([0, 0, 1])
    
    pnt = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
    quat = [tf.transform.rotation.x, tf.transform.rotation.y, 
            tf.transform.rotation.z, tf.transform.rotation.w]
    r = R.from_quat(quat)
    vec = - r.as_matrix() @ v2
    
    return pnt, vec
 
 
        
def gen_sphere():
    m = Marker() 
    m.type = 2  
    m.header.frame_id = "world"
        
    m.color.r = 255
    m.color.g = 255
    m.color.b = 255
    m.color.a = 0.5
        
    m.pose.position.x = ORIGIN[0]
    m.pose.position.y = ORIGIN[1]
    m.pose.position.z = ORIGIN[2]
        
    m.pose.orientation.x = 0
    m.pose.orientation.y = 1
    m.pose.orientation.z = 0
    m.pose.orientation.w = 0
        
    m.scale.x =PATH_RAD * 2
    m.scale.y =PATH_RAD * 2
    m.scale.z =PATH_RAD * 2
    
    return m    
    
   