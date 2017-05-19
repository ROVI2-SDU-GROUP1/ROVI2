#!/usr/bin/python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

import time

timestr = time.strftime("%Y%m%d-%H%M%S")

f = None
def callback(Point):
	print "Pose";
	f.write(str(Point.point.x) + "," + str(Point.point.y) + "," + str(Point.point.z) + "\n");
	f.flush();

def myhook():
	print "shutdown time!"
	f.close();

rospy.on_shutdown(myhook)


rospy.init_node('Logger_node', anonymous=True)
param_sub_pose_3d = rospy.get_param('~sub_3d_point', '/pose/3d')
param_filename = rospy.get_param('~filename', '/tmp/back.csv')
rospy.Subscriber(param_sub_pose_3d, PointStamped, callback)
f = open(param_filename, 'w')

rospy.spin()
