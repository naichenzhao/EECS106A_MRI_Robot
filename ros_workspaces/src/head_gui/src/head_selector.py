# TODO ros publisher for head movement
# establish a basis of the frame for the head
# have the robot path plan to the head
from scipy.spatial import KDTree
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import sys
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import csv
import trimesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import concurrent.futures
import time
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from stl import mesh
import matplotlib.tri as mtri
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import proj3d

# Load STL and extract points
mesh = trimesh.load('v17_head.stl')
points = np.vstack([mesh.vertices])

# Create a KD-tree for the original mesh vertices
kd_tree = KDTree(mesh.vertices)

# Reduce point cloud density
desired_number_of_points = 10000  # Adjust as needed
# reduced_points = points[np.random.choice(points.shape[0], size=desired_number_of_points, replace=False)]


def voxel_grid_downsampling(points, voxel_size):
    # Compute voxel indices
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)

    # Map voxel indices to unique IDs
    unique_voxels, indices = np.unique(
        voxel_indices, axis=0, return_inverse=True)

    # Compute mean coordinates for each voxel
    downsampled_points = np.zeros((unique_voxels.shape[0], 3))
    for i in range(unique_voxels.shape[0]):
        downsampled_points[i] = np.mean(points[indices == i], axis=0)

    return downsampled_points


# Apply Poisson Disk Sampling with a limit on the number of points
voxel_size = 10
min_distance = 0.1  # Adjust the minimum distance as needed
max_points = 100000   # Maximum number of points in the reduced point cloud
reduced_points = voxel_grid_downsampling(points, voxel_size)


class InteractivePlot(QMainWindow):
    def __init__(self, points, normals):
        super().__init__()
        self.points = points
        self.normals = normals
        self.initUI()

    def initUI(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # Create a matplotlib figure
        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.scatter(self.points[:, 0],
                        self.points[:, 1], self.points[:, 2])

        # Connect the mouse click event
        self.canvas.mpl_connect('button_press_event', self.on_click)

        self.show()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        # Project 3D data space to 2D data space
        x2, y2, _ = proj3d.proj_transform(
            self.points[:, 0], self.points[:, 1], self.points[:, 2], self.ax.get_proj())

        # Convert 2D data space to 2D screen space
        x2, y2 = np.array([self.ax.transData.transform((x, y))
                          for x, y in zip(x2, y2)]).T

        # Find the closest point
        distance = np.sqrt((x2 - event.x)**2 + (y2 - event.y)**2)
        closest_point_index = np.argmin(distance)
        closest_point = self.points[closest_point_index]
        closest_normal = self.normals[closest_point_index]

        # Clear the previous points and plot again
        self.ax.clear()
        self.ax.scatter(
            self.points[:, 0], self.points[:, 1], self.points[:, 2], s=1, c='blue')

        # Highlight the selected point in red
        self.ax.scatter(
            closest_point[0], closest_point[1], closest_point[2], s=10, c='red')

        # Re-draw the plot
        self.canvas.draw()

        print(f"Closest point: {closest_point}, Normal: {closest_normal}")


# Function to calculate normal for a single point
def calculate_normal(point_index):
    point = reduced_points[point_index]

    # Find the nearest vertex in the original mesh
    nearest_vertex_index = kd_tree.query(point)[1]
    nearest_vertex = mesh.vertices[nearest_vertex_index]

    # Find faces adjacent to the nearest vertex
    adjacent_faces = np.any(mesh.faces == nearest_vertex_index, axis=1)
    face_normals = mesh.face_normals[adjacent_faces]

    # Calculate the average normal
    if len(face_normals) > 0:
        normal = np.mean(face_normals, axis=0)
        normal /= np.linalg.norm(normal)
        return normal
    else:
        return np.array([0, 0, 0])


def load_mesh(file_path):
    return trimesh.load(file_path)


def calculate_normals(mesh, reduced_points):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        normals = list(executor.map(
            calculate_normal, range(len(reduced_points))))

    return np.array(normals)


def save_normals_to_csv(normals, file_path):
    with open(file_path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Normal_X', 'Normal_Y', 'Normal_Z'])  # Header
        writer.writerows(normals)


def visualize_point_cloud(reduced_points, normals, length_of_arrows=5):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(reduced_points[:, 0], reduced_points[:, 1],
               reduced_points[:, 2], s=1)

    for point, normal in zip(reduced_points, normals):
        ax.quiver(point[0], point[1], point[2], normal[0], normal[1],
                  normal[2], length=length_of_arrows, color='r')

    plt.show()


def main():
    # Parallel computation of normals
    start_time = time.time()
    # max_points = 100   # Maximum number of points in the reduced point cloud
    normals = calculate_normals(mesh, reduced_points)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

    # Save normals to CSV
    save_normals_to_csv(normals, 'normals.csv')

    # Visualization
    # visualize_point_cloud(reduced_points, normals)

    # Interactive GUI
    app = QApplication(sys.argv)
    ex = InteractivePlot(reduced_points, normals)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
