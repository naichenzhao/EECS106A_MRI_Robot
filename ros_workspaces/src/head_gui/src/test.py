from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import proj3d


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


def main():
    # Interactive GUI
    app = QApplication(sys.argv)
    # load from python binary file
    saved_reduced_points = np.load('points.npy')
    saved_normals = np.load('normals.npy')
    ex = InteractivePlot(saved_reduced_points, saved_normals)
    # ex = InteractivePlot(reduced_points, normals)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
