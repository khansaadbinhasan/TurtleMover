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

flag = 0

currentX = 0
currentY = 0

initX = 0
initY = 0

visitedXrange = []
visitedYrange = []

distFrontflag = 0
angZ = 0
angVel = 1

flag1 = 0

def laser_message( subMsg ):
	pubMsg = Twist()

	global distRightDel
	global distFrontDel
	global distLeftDel 
	global distRightold
	global distFrontold	
	global distLeftold
	global flag
	global visitedXrange
	global visitedYrange
	global distFrontflag
	global angZ
	global angVel
	global flag1


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
	print("angZ: ", angZ)

	distList = [distFrontnew, distLeftnew, distRightnew]
	maxDist = max(distList)

	rightAngle = (angZ < 0.1 and angZ > -0.1) or (angZ < (np.pi/2 + 0.1) and angZ > (np.pi/2 - 0.1)) or (angZ < -(np.pi/2 + 0.1) and angZ > -(np.pi/2 - 0.1) or (angZ < -(np.pi + 0.1) and angZ > -(np.pi - 0.1)) )

	print("(angZ < 0.1 and angZ > -0.1)",(angZ < 0.1 and angZ > -0.1))
	print("rightAngle",rightAngle)
	print("flag1==1",flag1==1)
	print("~rightAngle or flag1 == 1",bool(~rightAngle or flag1 == 1))

	if rightAngle:
		flag1 = 0

	if ~rightAngle or flag1 == 1:
		pubMsg.angular.z = angVel

	else:
		print("jher")
		if distFrontnew > 0.75 and distLeftnew > 0.75 and distRightnew > 0.75:
			
			if distFrontnew == maxDist or distFrontflag == 1:
				pubMsg.linear.x = 1
				distFrontflag = 1

			elif distRightnew == maxDist:
				angVel = 1
				flag1 = 1


			elif distLeftnew == maxDist:
				angVel = -1
				flag1 = 1


			if distFrontnew > 0.75 and distFrontnew < 0.8:
				distFrontflag = 0

		else:
			if distLeftnew <= 0.75:
				angVel = 1
				pubMsg.angular.z = angVel
				flag1 = 1

			else:
				angVel = -1
				pubMsg.angular.z = angVel
				flag1 = 1

	



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