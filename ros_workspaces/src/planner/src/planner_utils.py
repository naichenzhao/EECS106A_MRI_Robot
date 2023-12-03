import numpy as np
from scipy.spatial.transform import Rotation as R

def get_path_circular(p1, p2, origin = [0, 0, 0], res = 20):
    initial = get_angles(p1, origin)
    final = get_angles(p2, origin)
    
    radius = np.linspace(initial[0], final[0], res)
    theta = np.linspace(initial[1], final[1], res)
    phi = np.linspace(initial[2], final[2], res)
    
    points = np.zeros((res, 3))
    vecs = np.zeros((res, 3))
    
    for j in range(res):
        points[j, 0] = radius[j] * np.cos(theta[j]) * np.sin(phi[j]) + origin[0]
        points[j, 1] = radius[j] * np.sin(theta[j]) * np.sin(phi[j]) + origin[1]
        points[j, 2] = radius[j] * np.cos(phi[j]) + origin[2]
        
        vecs[j, 0] = - (points[j, 0] - origin[0])
        vecs[j, 1] = - (points[j, 1] - origin[1])
        vecs[j, 2] = - (points[j, 2] - origin[2])
        
    return points, vecs

def get_path_linear(p1, p2, res = 5):
    points = np.zeros((res, 3))
    points[:, 0] = np.linspace(p1[0], p2[0], res)
    points[:, 1] = np.linspace(p1[1], p2[1], res)
    points[:, 2] = np.linspace(p1[2], p2[2], res)
    
    vecs = [(p2 - p1)/ np.linalg.norm(p2 - p1)] * res
    return points, vecs


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

def pick_point(p1, p2):
    if p1[0] > 0 and abs(p1[1]) < 0.08 and p1[2] > 0:
        return p1
    return p2

def vect_to_quat(v):
    v2 = np.array([0, 1, 0])
    
    rot_mat = R.align_vectors(-v, v2)
    r = R.from_matrix(rot_mat)
    
    return r.as_quat()

def transform_to_vec(tf):
    v2 = np.array([0, 1, 0])
    
    pnt = [tf.transform.translation.y, tf.transform.translation.x, tf.transform.translation.z]
    quat = [tf.transform.rotation.x, tf.transform.rotation.y, 
            tf.transform.rotation.z, tf.transform.rotation.w]
    r = R.from_quat(quat)
    vec = r.as_matrix() @ v2
    
    return pnt, vec
        
    
    

# Plotting Functions 
def plot_vec(ax, p, v, colour = "red" ):
    vals = np.zeros((2, 3))
    vals[0] = p
    vals[1] = p + (v * 0.01)/ np.linalg.norm(v)
    ax.scatter3D(p[0], p[1], p[2], color = colour )
    ax.plot(vals[:, 0], vals[:, 1], vals[:, 2], color = colour )

def plot_sphere(ax, r, center = [0, 0, 0], colour = "blue"):
    u, v = np.mgrid[-np.pi/2:np.pi/2:20j, 0:-np.pi/2:10j]
    x = r * np.cos(u)*np.sin(v) + center[0]
    y = r * np.sin(u)*np.sin(v) + center[1]
    z = r * np.cos(v) + center[2]
    ax.plot_surface(x, y, z, color=colour, alpha = 0.5)
    
def plot_cube(ax, p1, p2):
    vertices = [
            [p1[0], p1[1], p1[2]],
            [p2[0], p1[1], p1[2]],
            [p2[0], p2[1], p1[2]],
            [p1[0], p2[1], p1[2]],
            [p1[0], p1[1], p2[2]],
            [p2[0], p1[1], p2[2]],
            [p2[0], p2[1], p2[2]],
            [p1[0], p2[1], p2[2]]
    ]
    # Define cube edges
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
 
    # Plot the cube
    for edge in edges:
        x = [vertices[edge[0]][0], vertices[edge[1]][0]]
        y = [vertices[edge[0]][1], vertices[edge[1]][1]]
        z = [vertices[edge[0]][2], vertices[edge[1]][2]]
        ax.plot(x, y, z, 'y--', alpha = 0.5)