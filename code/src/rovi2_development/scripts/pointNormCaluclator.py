#!/usr/bin/python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

import time

timestr = time.strftime("%Y%m%d-%H%M%S")
f = open('/tmp/' + timestr + ".csv", 'w')

def calcNorm():
	f.write("Calculating norm of last two points - ");
	f.flush();
	p1 = myList(0)
	p2 = myList(1)
	pointNorm = sqrt( (p1.point.x - p2.point.x)**2 + (p1.point.y - p2.point.y)**2 + (p1.point.z - p2.point.z)**2 )
	f.write("Distance: " + str(pointNorm));
	f.flush();

def callback(Point):
	if len(myList) == 0:
		myList.append(Point)
		f.write("I need one more point");
		f.flush();
		return
	elif len(myList) == 1:
		myList.append(Point)
	elif len(myList) == 2:
		myList.append(Point)
		myList.pop(0)

	calcNorm()

def myhook():
	print "shutdown time!"
	f.close();

myList = []

rospy.on_shutdown(myhook)

rospy.init_node('Logger_node', anonymous=True)

param_sub_pose_3d = rospy.get_param('~sub_3d_point', '/pose/3d')

sub_3d_point = rospy.Subscriber(param_sub_pose_3d, PointStamped, callback)

rospy.spin()
