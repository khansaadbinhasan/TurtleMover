#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, traceback
from nav_msgs.msg import Odometry



pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

distRightold, distLeftold, distFrontold = 0, 0, 0
distRightDel, distLeftDel, distFrontDel = 0, 0, 0

flag0 = 0
flag1 = 0
flag2 = 0

angVel = 1
sumAng = 0
count = 0


def laser_message( subMsg ):
	pubMsg = Twist()

	global distRightDel
	global distFrontDel
	global distLeftDel 
	global distRightold
	global distFrontold	
	global distLeftold
	global flag0
	global flag1
	global flag2
	global angVel
	global sumAng
	global count

	# These statements take care of the nan values: 
	# The values are nan for
	if ~np.isnan(subMsg.ranges[0]):
		distRightnew = subMsg.ranges[0]
		distRightDel = distRightnew - distRightold

	elif np.isnan(subMsg.ranges[0]) and distRightDel >= 0:
		distRightnew = 11

	elif np.isnan(subMsg.ranges[0]) and distRightDel < 0:
		distRightnew = 0


	if ~np.isnan(subMsg.ranges[len(subMsg.ranges)/2]):
		distFrontnew = subMsg.ranges[len(subMsg.ranges)/2] 
		distFrontDel = distFrontnew - distFrontold

	elif np.isnan(subMsg.ranges[len(subMsg.ranges)/2]) and distFrontDel >= 0:
		distFrontnew = 11

	elif np.isnan(subMsg.ranges[len(subMsg.ranges)/2]) and distFrontDel < 0:
		distFrontnew = 0


	if ~np.isnan(subMsg.ranges[-1]):
		distLeftnew = subMsg.ranges[-1]
		distLeftDel = distLeftnew - distLeftold

	elif np.isnan(subMsg.ranges[-1]) and distLeftDel >= 0:
		distLeftnew = 11

	elif np.isnan(subMsg.ranges[-1]) and distLeftDel < 0:
		distLeftnew = 0



	print("\n"*3)
	print("distRight: ", distRightnew)
	print("distFront: ", distFrontnew)
	print("distLeft: ", distLeftnew)

	distList = [distFrontnew, distLeftnew, distRightnew]
	maxDist = max(distList)

	angVel = 0
	xVel = 0

	print("flag1: ", flag1)
	print("flag2: ", flag2)

	if distFrontnew > 0.5 and distLeftnew > 0.5 and distRightnew > 0.5:
		
		if distFrontnew < 0.75 or distLeftnew < 0.75 or distRightnew < 0.75:
			xVel = 0

			if distLeftnew < 0.75:
				angVel = 1
				flag1 = 1

			elif flag1 != 1:
				angVel = -1
				flag2 = 1

			else:
				angVel = 1
				flag1 = 1


		elif distFrontnew == maxDist or flag0 == 1:
			xVel = 1
			flag0 = 1

		elif abs(distRightnew - distLeftnew) < 0.25 and flag1 != 1:
			angVel = -1

		elif abs(distRightnew - distFrontnew) < 0.25:
			xVel = 1

		elif abs(distLeftnew - distFrontnew) < 0.25:
			xVel = -1


		elif distRightnew == maxDist and flag1 == 1:
			xVel = -1

		elif distRightnew == maxDist:
			angVel = -1

		elif distLeftnew == maxDist and flag2 == 1:
			xVel = -0.1

		elif distLeftnew == maxDist:
			angVel = 1

		if distFrontnew > 0.5 and distFrontnew < 0.6:
			flag0 = 0

		if distRightnew >= 1.00:
			flag2 = 0

		if distLeftnew >= 1.00:
			flag1 = 0


		sumAng = sumAng + angVel
		count = count + 1

		print("sumAng: ", sumAng)

		if count % 25 == 0 and sumAng <= 10 and sumAng >= -10:
			angVel = 1


# angVel = -1
	# xVel = 0

	pubMsg.angular.z = angVel
	pubMsg.linear.x = xVel

	distRightold, distLeftold, distFrontold = distRightnew, distLeftnew, distFrontnew
	
	pub.publish(pubMsg)


def odom_message( subMsg ):
	global currentX
	global currentY
	global angZ

	currentX = subMsg.pose.pose.position.x
	currentY = subMsg.pose.pose.position.y
	angZ = subMsg.pose.pose.orientation.z

def run():
	rospy.init_node("demoMover", anonymous=True)
	rospy.Subscriber("/scan", LaserScan, laser_message)	
	rospy.Subscriber("/odom", Odometry, odom_message)	

	rospy.spin()



if __name__ == '__main__':

	try:
		run()

	except rospy.ROSInterruptException:
		pass