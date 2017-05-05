#!/usr/bin/python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from caros_control_msgs.msg import RobotState
import time

timestr = time.strftime("%Y%m%d-%H%M%S")
f = open('/tmp/' + timestr + ".csv", 'w')

def callback(Q, Pose):
	print "Combined Q and pose";
	f.write(str(Q.q.data) + "," + str(Pose.point.x) + "," + str(Pose.point.y) + "," + str(Pose.point.z) + "\n");
	f.flush();

def myhook():
	print "shutdown time!"
	f.close();

rospy.on_shutdown(myhook)

rospy.init_node('Logger_node', anonymous=True)
param_sub_robot_state = rospy.get_param('~sub_robot_state', '/q_state_processed')
param_sub_pose_3d = rospy.get_param('~sub_3d_point', '/pose/3d')

sub_q_processed = message_filters.Subscriber(param_sub_robot_state, RobotState)
sub_3d_point = message_filters.Subscriber(param_sub_pose_3d, PointStamped)

ts = message_filters.TimeSynchronizer([sub_q_processed, sub_3d_point], 10)
ts.registerCallback(callback)
rospy.spin()
