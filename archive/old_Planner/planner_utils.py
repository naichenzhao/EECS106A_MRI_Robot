import numpy as np
from planner import PATH_RAD, ORIGIN
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

def get_path_circular(p1, p2, origin = [0, 0, 0], res = 20):
    initial = get_angles(p1, origin)
    final = get_angles(p2, origin)
    
    radius = np.linspace(initial[0], final[0], res + 1)
    theta = np.linspace(initial[1], final[1], res + 1)
    phi = np.linspace(initial[2], final[2], res + 1)
    
    points = np.zeros((res + 1, 3))
    vecs = np.zeros((res + 1, 3))
    
    for j in range(res + 1):
        points[j, 0] = radius[j] * np.cos(theta[j]) * np.sin(phi[j]) + origin[0]
        points[j, 1] = radius[j] * np.sin(theta[j]) * np.sin(phi[j]) + origin[1]
        points[j, 2] = radius[j] * np.cos(phi[j]) + origin[2]
        
        vecs[j, 0] = (points[j, 0] - origin[0])
        vecs[j, 1] = 0 #(points[j, 1] - origin[1])
        vecs[j, 2] = 0 #(points[j, 2] - origin[2])
        
    return points, vecs



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



def pick_point(p1, p2, v):
    len1 = np.linalg.norm(np.array(p1) - np.array(v))
    len2 = np.linalg.norm(np.array(p2) - np.array(v))
    if len1 < len2:
        return p1
    return p2



def get_rollpitch(v1): 
    yaw = np.arcsin(v1[1]/np.linalg.norm(v1)) - np.pi/2
    pitch =  np.arcsin(v1[2]/np.linalg.norm(v1))
    
    return yaw, pitch



def vect_to_quat(v):
    yaw, pitch = get_rollpitch(v)
    # return quaternion_from_euler(0 - np.pi/2, pitch + np.pi/2, yaw + np.pi/2)
    return quaternion_from_euler(3*np.pi/2, pitch, yaw)



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
    
    

# Plotting Functions 
def plot_vec(ax, p, v, colour = "red" ):
    vals = np.zeros((2, 3))
    vals[0] = p
    vals[1] = p + (v * 0.01)/ np.linalg.norm(v)
    # ax.scatter3D(p[0], p[1], p[2], color = colour )
    ax.plot(vals[:, 0], vals[:, 1], vals[:, 2], color = colour )



def plot_sphere(ax, r, center = [0, 0, 0], colour = "blue"):
    u, v = np.mgrid[-np.pi/2:np.pi/2:20j, 0:-np.pi/2:10j]
    x = r * np.cos(u)*np.sin(v) + center[0]
    y = r * np.sin(u)*np.sin(v) + center[1]
    z = r * np.cos(v) + center[2]
    ax.plot_surface(x, y, z, color=colour, alpha = 0.5)



def in_circle(p):
    num_p = np.array(p) - ORIGIN
    return np.linalg.norm(num_p) <= PATH_RAD
    
    
    