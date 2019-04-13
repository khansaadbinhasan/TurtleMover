#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, traceback
from nav_msgs.msg import Odometry



pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=5)

dist = {
	"Rightold" : 0,
	"Leftold" : 0,
	"Frontold" : 0,
	"RightDel": 0,
	"LeftDel": 0,
	"FrontDel" : 0,
	"Rightnew": 0,
	"Leftnew": 0,
	"Frontnew" : 0
}

flag = [False, False, False, False, False, False, False]

pos = {
	"currentX" : 0,
	"currentY" : 0,
	"initX" : 0,
	"initY" : 0
}

angVel = 0

def laser_message( subMsg ):
	pubMsg = Twist()


	xVel, angVel = path_algo( subMsg )

	pubMsg.angular.z = angVel
	pubMsg.linear.x = xVel
	
	pub.publish(pubMsg)


def odom_message( subMsg ):
	global pos
	global flag


	pos["currentX"] = subMsg.pose.pose.position.x
	pos["currentY"] = subMsg.pose.pose.position.y

	if flag[6] == False:
		flag[6] = True
		pos["initX"] = pos["currentX"]
		pos["initY"] = pos["currentY"]


	angZ = subMsg.pose.pose.orientation.z


def path_algo( subMsg ):
	global dist
	global flag
	global pos

	

	# These statements take care of the nan values: 
	# The values are nan for out of range of (0.5,10)

	if ~np.isnan(subMsg.ranges[0]):
		dist["Rightnew"] = subMsg.ranges[0]
		dist["RightDel"] = dist["Rightnew"] - dist["Rightold"]

	elif np.isnan(subMsg.ranges[0]):
		
		# if dist["RightDel"] >= 0:
		dist["Rightnew"] = 11

		# else:
		# 	dist["Rightnew"] = 0


	if ~np.isnan(subMsg.ranges[len(subMsg.ranges)/2]):
		dist["Frontnew"] = subMsg.ranges[len(subMsg.ranges)/2] 
		dist["FrontDel"] = dist["Frontnew"] - dist["Frontold"]

	elif np.isnan(subMsg.ranges[len(subMsg.ranges)/2]):
		# if dist["FrontDel"] >= 0:
		dist["Frontnew"] = 11

		# else:
		# 	dist["Frontnew"] = 0


	if ~np.isnan(subMsg.ranges[-1]):
		dist["Leftnew"] = subMsg.ranges[-1]
		dist["LeftDel"] = dist["Leftnew"] - dist["Leftold"]

	elif np.isnan(subMsg.ranges[-1]):
		# if dist["LeftDel"] >= 0:
		dist["Leftnew"] = 11

		# else:
		# 	dist["Leftnew"] = 0



	distList = [dist["Frontnew"], dist["Leftnew"], dist["Rightnew"]]
	maxDist = max(distList)

	angVel = 0
	xVel = 0

	left = -1
	right = 1
	front = 1
	back = -0.1
	rest = 0


	X = pos["currentX"] - pos["initX"]
	Y = pos["currentY"] - pos["initY"]


	print("\n"*3)
	print("distRight: ", dist["Rightnew"])
	print("distFront: ", dist["Frontnew"])
	print("distLeft: ", dist["Leftnew"])

	print("initX: ", pos["initX"])
	print("initY: ", pos["initY"])

	print("X: ", X)
	print("Y: ", Y)

	print("\n"*3)

	if dist["Frontnew"] > 0.5 and dist["Leftnew"] > 0.5 and dist["Rightnew"] > 0.5:
		
		if dist["Frontnew"] < 0.75 or dist["Leftnew"] < 0.75 or dist["Rightnew"] < 0.75:

			xVel = rest


			if dist["Frontnew"] < 0.75 or flag[4] == True:
				xVel = -0.1
				flag[4] = True
				angVel = right

				print("going back")
				print("rotating left")

				if ( Y < 3 and Y > -3 and X < 3 ) or flag[5] == True:
					angVel = left
					flag[5] = True

					print("rotating right")

				

			elif dist["Rightnew"] < 0.75 or flag[2] == True:
				angVel = left
				flag[2] = True
				flag[1] = False
				flag[0] = False

				print("rotating left")

			elif dist["Leftnew"] < 0.75 or flag[1] == True:
				angVel = right
				flag[0] = False
				flag[1] = True	
				flag[2] = False

				print("rotating right")

		elif dist["Frontnew"] == maxDist or flag[0] == True:
			xVel = front
			flag[0] = True
			flag[1] = False
			flag[2] = False

			print("going forward")

		elif dist["Leftnew"] == maxDist or flag[1] == True:
			angVel = right
			flag[0] = False
			flag[1] = True
			flag[2] = False

			print("rotating right")

		elif dist["Rightnew"] == maxDist or flag[2] == True:
			angVel = left
			flag[0] = False
			flag[1] = False
			flag[2] = True

			print("rotating left")


		if dist["Frontnew"] > 0.5 and dist["Frontnew"] < 0.75:
			flag[0] = False


	dist["Rightold"], dist["Leftold"], dist["Frontold"] = dist["Rightnew"], dist["Leftnew"], dist["Frontnew"]

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