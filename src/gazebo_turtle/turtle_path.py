#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, traceback
from nav_msgs.msg import Odometry



pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=5)

distRightold, distLeftold, distFrontold = 0, 0, 0
distRightDel, distLeftDel, distFrontDel = 0, 0, 0

flag0 = False
flag1 = False
flag2 = False
flag3 = False
flag4 = False
flag5 = False
flag6 = False

# angVel = 1
sumAng = 0
count = 0

angZ = 0
initAngZ = 0

currentX = 0
currentY = 0

initX = 0
initY = 0

def laser_message( subMsg ):
	pubMsg = Twist()


	xVel, angVel = path_algo( subMsg )

	# if delAngZ < 0.3:
	# 	angVel = -1
	# xVel = 0

	pubMsg.angular.z = angVel
	pubMsg.linear.x = xVel
	
	pub.publish(pubMsg)


def odom_message( subMsg ):
	global currentX
	global currentY
	global angZ
	global flag6

	if flag6 == False:
		flag6 = True
		initX = currentX
		initY = currentY

	currentX = subMsg.pose.pose.position.x
	currentY = subMsg.pose.pose.position.y
	angZ = subMsg.pose.pose.orientation.z


def path_algo( subMsg ):
	global distRightDel
	global distFrontDel
	global distLeftDel 
	global distRightold
	global distFrontold	
	global distLeftold
	global flag0
	global flag1
	global flag2
	global flag3
	global flag4
	global flag5
	# global angVel
	global sumAng
	global count
	global rotatedAng
	global angZ
	global initAngZ
	global currentX
	global currentY

	# These statements take care of the nan values: 
	# The values are nan for
	if ~np.isnan(subMsg.ranges[0]):
		distRightnew = subMsg.ranges[0]
		distRightDel = distRightnew - distRightold

	elif np.isnan(subMsg.ranges[0]):
		
		# if distRightDel >= 0:
		distRightnew = 11

		# else:
		# 	distRightnew = 0


	if ~np.isnan(subMsg.ranges[len(subMsg.ranges)/2]):
		distFrontnew = subMsg.ranges[len(subMsg.ranges)/2] 
		distFrontDel = distFrontnew - distFrontold

	elif np.isnan(subMsg.ranges[len(subMsg.ranges)/2]):
		# if distFrontDel >= 0:
		distFrontnew = 11

		# else:
		# 	distFrontnew = 0


	if ~np.isnan(subMsg.ranges[-1]):
		distLeftnew = subMsg.ranges[-1]
		distLeftDel = distLeftnew - distLeftold

	elif np.isnan(subMsg.ranges[-1]):
		# if distLeftDel >= 0:
		distLeftnew = 11

		# else:
		# 	distLeftnew = 0



	distList = [distFrontnew, distLeftnew, distRightnew]
	maxDist = max(distList)

	angVel = 0
	xVel = 0

	left = -1
	right = 1
	front = 1
	back = -0.1
	rest = 0

	delAngZ = abs(angZ - initAngZ)

	X = currentX - initX
	Y = currentY - initY


	print("\n"*3)
	print("distRight: ", distRightnew)
	print("distFront: ", distFrontnew)
	print("distLeft: ", distLeftnew)
	
	print("X: ", X)
	print("Y: ", Y)

	print("\n"*3)

	if distFrontnew > 0.5 and distLeftnew > 0.5 and distRightnew > 0.5:
		
		if distFrontnew < 0.75 or distLeftnew < 0.75 or distRightnew < 0.75:

			xVel = rest

			if count == 1:
				initAngZ = angZ


			elif distFrontnew < 0.75 or flag4 == True:
				xVel = -0.1
				flag4 = True
				angVel = right

				print("going back")
				print("rotating left")

				if ( Y > 0 and X > 5) or flag5 == True:
					angVel = left
					flag5 = True

					print("rotating right")

				

			elif distRightnew < 0.75 or flag2 == True:
				angVel = left
				flag2 = True
				flag1 = False
				flag0 = False

				print("rotating left")

			elif distLeftnew < 0.75 or flag1 == True:
				angVel = right
				flag0 = False
				flag1 = True	
				flag2 = False

				print("rotating right")

		elif distFrontnew == maxDist or flag0 == True:
			xVel = front
			flag0 = True
			flag1 = False
			flag2 = False

			print("going forward")

		elif distLeftnew == maxDist or flag1 == True:
			angVel = right
			flag0 = False
			flag1 = True
			flag2 = False

			print("rotating right")

		elif distRightnew == maxDist or flag2 == True:
			angVel = left
			flag0 = False
			flag1 = False
			flag2 = True

			print("rotating left")


		if distFrontnew > 0.5 and distFrontnew < 0.75:
			flag0 = False


	distRightold, distLeftold, distFrontold = distRightnew, distLeftnew, distFrontnew

	return xVel, angVel

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