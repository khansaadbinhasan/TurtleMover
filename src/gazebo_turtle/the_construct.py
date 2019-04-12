#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)


def poseMessageReceived( msg ):
	print("\n"*3)
	print("value at 0 degrees: ", msg.ranges[0])
	print("value at 90 degrees: ", msg.ranges[len(msg.ranges)/2])
	print("value at 180 degrees: ", msg.ranges[-1])


def run():
	rospy.init_node("demoMover", anonymous=True)
	sub = rospy.Subscriber("/scan", LaserScan, poseMessageReceived)	

	rospy.spin()



if __name__ == '__main__':

	try:
		run()

	except rospy.ROSInterruptException:
		pass