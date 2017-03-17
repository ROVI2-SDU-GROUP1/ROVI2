#!/usr/bin/env python

import sys
import rospy
from caros_control_msgs.srv import SerialDeviceMoveLin
from caros_control_msgs.srv import SerialDeviceMovePtp
from caros_common_msgs.msg import Q

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Float64MultiArray

#float32[] speeds
#float32[] blends


#from beginner_tutorials.srv import *

if __name__ == "__main__":
#    rospy.wait_for_service('/ur_simple_demo_node/caros_serial_device_service_interface/move_lin')
    rospy.wait_for_service('/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp')
    try:
        Handle = rospy.ServiceProxy('/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp', SerialDeviceMovePtp)
	data = []

#	data.append(Transform(Vector3(0,-0.19145,0.60106), Quaternion(0,0,0,1)));
#	data.append(Q(0,0,0,0,0,0))
#	for i in range(6):
#		qs.data.append( Float64(0) )

	data.append(Q([0,4*3.14,0,0,0,0]))


	speeds = []
	speeds.append(0.1)

	blends = []
	blends.append(0.1);
        resp1 = Handle(data, speeds, blends)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
