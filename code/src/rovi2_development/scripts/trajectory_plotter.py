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
        plt.show()
        last_plot = None
        last_points = None

        while(True):
            with self.lock:
                if(last_plot is not None): last_plot.remove()
                if(last_points is not None):
                    for p in last_points: p.remove()
                if(len(self.filtered_trajectories) == 0):
                    time.sleep(0.1)
                    continue
                if(len(self.raw_points) == 0):
                    time.sleep(0.1)
                    continue

                last_traj = self.filtered_trajectories[-1]
                t = np.linspace(- 5,  5, 1000)
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
    rospy.init_node('node_name')
    rospy.Subscriber("/pose/3d_true", PointStamped, t_plotter.raw_points_callback)
    rospy.Subscriber("/pose/3d", PointStamped, t_plotter.noisy_points_callback)
    rospy.Subscriber("/pose/parameter", Trajectory3D, t_plotter.trajectory_callback)

    rospy.spin()




if __name__ == "__main__":
    __main__()
