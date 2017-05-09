#!/usr/bin/python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

import time

timestr = time.strftime("%Y%m%d-%H%M%S")
f = open('/tmp/' + timestr + ".csv", 'w')

def callback(Point, Robot):
	print "Combined Q and pose";
	f.write(str(Point.point.x) + "," + str(Point.point.y) + "," + str(Point.point.z) \
	 + "," + str(Robot.transform.translation.x) + "," + str(Robot.transform.translation.y) \
	 + "," + str(Robot.transform.translation.z) + "\n");
	f.flush();

def myhook():
	print "shutdown time!"
	f.close();

rospy.on_shutdown(myhook)

rospy.init_node('Logger_node', anonymous=True)
param_sub_robot_state = rospy.get_param('~sub_robot_state', '/q_state_processed')
param_sub_pose_3d = rospy.get_param('~sub_3d_point', '/pose/3d')

sub_transform = message_filters.Subscriber(param_sub_robot_state, TransformStamped)
sub_3d_point = message_filters.Subscriber(param_sub_pose_3d, PointStamped)

ts = message_filters.TimeSynchronizer([sub_3d_point,sub_transform], 10)
ts.registerCallback(callback)
rospy.spin()
