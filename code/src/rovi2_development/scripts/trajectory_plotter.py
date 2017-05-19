#!/usr/bin/python2
#Small node for plotting trajectories otputed
import threading
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import time
from rovi2_development.msg import Trajectory3D
from geometry_msgs.msg import PointStamped
import random

from mpl_toolkits.mplot3d import axes3d
from matplotlib.patches import Circle, PathPatch
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from mpl_toolkits.mplot3d import art3d
import numpy as np

def plot_vector(fig, orig, v, color='blue'):
   ax = fig.gca(projection='3d')
   orig = np.array(orig); v=np.array(v)
   ax.quiver(orig[0], orig[1], orig[2], v[0], v[1], v[2],color=color)
   ax.set_xlim(0,10);ax.set_ylim(0,10);ax.set_zlim(0,10)
   ax = fig.gca(projection='3d')
   return fig

def rotation_matrix(d):
    sin_angle = np.linalg.norm(d)
    if sin_angle == 0:return np.identity(3)
    d /= sin_angle
    eye = np.eye(3)
    ddt = np.outer(d, d)
    skew = np.array([[    0,  d[2],  -d[1]],
                  [-d[2],     0,  d[0]],
                  [d[1], -d[0],    0]], dtype=np.float64)

    M = ddt + np.sqrt(1 - sin_angle**2) * (eye - ddt) + sin_angle * skew
    return M

def pathpatch_2d_to_3d(pathpatch, z, normal):
    if type(normal) is str: #Translate strings to normal vectors
        index = "xyz".index(normal)
        normal = np.roll((1.0,0,0), index)

    normal /= np.linalg.norm(normal) #Make sure the vector is normalised
    path = pathpatch.get_path() #Get the path and the associated transform
    trans = pathpatch.get_patch_transform()

    path = trans.transform_path(path) #Apply the transform

    pathpatch.__class__ = art3d.PathPatch3D #Change the class
    pathpatch._code3d = path.codes #Copy the codes
    pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color

    verts = path.vertices #Get the vertices in 2D

    d = np.cross(normal, (0, 0, 1)) #Obtain the rotation vector
    M = rotation_matrix(d) #Get the rotation matrix

    pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])

def pathpatch_translate(pathpatch, delta):
    pathpatch._segment3d += delta

def plot_plane(ax, point, normal, size=10, color='y'):
    p = Circle((0, 0), size, facecolor = color, alpha = .2)
    ax.add_patch(p)
    pathpatch_2d_to_3d(p, z=0, normal=normal)
    pathpatch_translate(p, (point[0], point[1], point[2]))


class Trajectory_Plotter:
    def __init__(self):
        self.lock = threading.RLock()
        self.plotter_thread = threading.Thread(target = self.plotter)
        self.plotter_thread.daemon = True
        self.plotter_thread.start()
        self.filtered_trajectories = []
        self.noisy_points = []
        self.raw_points = []
    def plotter(self):
        plt.ion()
        fig = plt.figure(figsize=(16, 8), dpi=100,)
        ax = fig.add_subplot(111, projection='3d')
        plt.axis('equal')
        plt.show()
        last_plot = None
        last_points = None
        plane_point  = np.array([0, -0.3, 0])
        plane_normal = np.array([0.000001, 1, 0.000001])
        plot_plane(ax, plane_point, plane_normal)
        while(len(self.raw_points) < 200 or True):
            with self.lock:
                if(last_plot is not None): last_plot.remove()
                if(last_points is not None):
                    for p in last_points: p.remove()
                if(len(self.filtered_trajectories) == 0):
                    continue
                #if(len(self.raw_points) == 0):
                #    continue

                last_traj = self.filtered_trajectories[-1]
                t = np.linspace(- 1,  1, 1000)
                x = 0.5 * last_traj.acc.x * t * t + \
                    last_traj.vel.x * t + last_traj.pos.x
                y = 0.5 * last_traj.acc.y * t * t + \
                    last_traj.vel.y * t + last_traj.pos.y
                z = 0.5 * last_traj.acc.z * t * t + \
                    last_traj.vel.z * t + last_traj.pos.z
                last_plot, = ax.plot(x, y, z, label = 'estimated trajectory')
                last_points = []
                tmp = ax.scatter( [point.point.x for point in self.raw_points], \
                            [point.point.y for point in self.raw_points], \
                            [point.point.z for point in self.raw_points],
                             c = 'r',  marker = '^'
                            )
                last_points.append(tmp)
                tmp = ax.scatter( [point.point.x for point in self.noisy_points], \
                            [point.point.y for point in self.noisy_points], \
                            [point.point.z for point in self.noisy_points],
                             c = 'g',  marker = '*'
                            )
                last_points.append(tmp)
                """ax.set_xlim(self.raw_points[-1].point.x - 20, \
                    self.raw_points[-1].point.x + 20)
                ax.set_ylim(self.raw_points[-1].point.y - 20, \
                    self.raw_points[-1].point.y + 20)
                ax.set_zlim(self.raw_points[-1].point.z - 20, \
                    self.raw_points[-1].point.z + 20)"""


            plt.pause(0.05)
        while(True):
            plt.pause(0.05)
    def raw_points_callback(self, the_point):
        with self.lock:
            self.raw_points.append(the_point)

    def noisy_points_callback(self, the_point):
        with self.lock:
            self.noisy_points.append(the_point)

    def trajectory_callback(self, the_traject):
        with self.lock:
            self.filtered_trajectories.append(the_traject)


def __main__():
    t_plotter = Trajectory_Plotter()

    sub_parameter = rospy.get_param("~sub_parameter", "/pose/parameter")

    rospy.init_node('node_name')
    rospy.Subscriber("/pose/3d_true", PointStamped, t_plotter.raw_points_callback,)
    rospy.Subscriber("/pose/3d", PointStamped, t_plotter.noisy_points_callback)
    rospy.Subscriber(sub_parameter, Trajectory3D, t_plotter.trajectory_callback)

    rospy.spin()

if __name__ == "__main__":
    __main__()
