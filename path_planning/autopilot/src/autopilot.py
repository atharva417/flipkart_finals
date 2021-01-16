#!/usr/bin/env python
# ROS python API
import rospy
import time

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from utils.offboard import mavcon
import numpy as np
#import pcl
#import pcl_helper
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import tf2_ros
from frame_detection.msg import frame

class autopilot:
	
	def __init__(self):
		'''
		self.isright = False
		self.isleft = False
		'''
		self.currentframe = 0
		self.nextframe = 1
		self.rate = rospy.Rate(20.0)
		self.framecount = 0
		self.wplist = []
		self.wplistpr = []
		rospy.Subscriber('/frame_centers/position', frame, self.detectFrames)
		rospy.Subscriber('/mavros/global_position/local', Odometry, self.poseCallback)
		print('Class Instantiated with Subscribers')
		self.mvc = mavcon()

	def detectFrames(self, msg):	
		self.wplist = msg.centers
		self.firstframex = msg.centers[0].x

	def getFrameNum(self): 
		if (self.currentposex < self.firstframex) :
			self.currentframe = 0
		elif ((self.currentposex > self.firstframex) and (self.currentposex < self.firstframex + 1)) :
			self.currentframe = 1
		elif ((self.currentposex > (self.firstframex + 1)) and (self.currentposex < (self.firstframex + 2))) :
			self.currentframe = 2
		elif ((self.currentposex > (self.firstframex + 2)) and (self.currentposex < (self.firstframex + 3))) :
			self.currentframe = 3
		elif ((self.currentposex > (self.firstframex + 3)) and (self.currentposex < (self.firstframex + 4))) :
			self.currentframe = 4
		elif ((self.currentposex > (self.firstframex + 4)) and (self.currentposex < (self.firstframex + 5))) :
			self.currentframe = 5
		elif ((self.currentposex > (self.firstframex + 5)) and (self.currentposex < (self.firstframex + 6))) :
			self.currentframe = 6
		elif ((self.currentposex > (self.firstframex + 6)) and (self.currentposex < (self.firstframex + 7))) :
			self.currentframe = 7
		elif ((self.currentposex > (self.firstframex + 7)) and (self.currentposex < (self.firstframex + 8))) :
			self.currentframe = 8
		elif ((self.currentposex > (self.firstframex + 8)) and (self.currentposex < (self.firstframex + 9))) :
			self.currentframe = 9
		elif ((self.currentposex > (self.firstframex + 9)) and (self.currentposex < (self.firstframex + 10))) :
			self.currentframe = 10
		elif ((self.currentposex > (self.firstframex + 10)) and (self.currentposex < (self.firstframex + 11))) :
			self.currentframe = 11
		elif ((self.currentposex > (self.firstframex + 11)) and (self.currentposex < (self.firstframex + 12))) :
			self.currentframe = 12
		elif ((self.currentposex > (self.firstframex + 12)) and (self.currentposex < (self.firstframex + 13))) :
			self.currentframe = 13
		elif ((self.currentposex > (self.firstframex + 13)) and (self.currentposex < (self.firstframex + 14))) :
			self.currentframe = 14
		elif ((self.currentposex > (self.firstframex + 14)) and (self.currentposex < (self.firstframex + 15))) :
			self.currentframe = 15
		#print('Frame Classified according to Position')


	def incrementCurrentFrame(self):
		self.framecount = self.framecount + 1

	def explore(self):
		global mvc
		exploretimestart = time.time()
		self.nextframe = self.currentframe + 1
		nextframex, nextframey, nextframez = self.wplist[self.framecount].x, self.wplist[self.framecount].y, self.wplist[self.framecount].z  
		
		if ((nextframex == 0) and (nextframey == 0) and (nextframez == 0)):
			if self.currentposey < 0:
				self.mvc.gotopose(self.currentposex, 1.5, 3.275)
				self.mvc.gotopose(self.currentposex - 0.5, self.currentposey, 3.275)
				self.mvc.gotopose(self.currentposex + 0.5, self.currentposey, 3.275)
				print('reexploring')

			if self.currentposey > 0:
				self.mvc.gotopose(self.currentposex, -1.5, 3.275)
				self.mvc.gotopose(self.currentposex - 0.5, self.currentposey, 3.275)
				self.mvc.gotopose(self.currentposex + 0.5, self.currentposey, 3.275)
				print('reexploring')
		else:
			self.mvc.gotopose(nextframex - 0.5, nextframey, 3.275)
			#print('No need to reexplore')


			
		
		#rate.sleep()

		exploretimeend = time.time()
		#print('Explore Computation Time = ', exploretimeend - exploretimestart, 'seconds')

	def poseCallback(self, msg):
		self.currentposex = msg.pose.pose.position.x
		self.currentposey = msg.pose.pose.position.y
	
	'''
	def planner(self, pointlist):
		self.path = []
		for point in pointlist:
			x = point[0] - 0.5
			y = point[1]
			z = point[2]
			self.path.append([x, y, z])
		return self.path
	'''	

def main():
	maintimestart = time.time()
	rospy.init_node('autopilot')
	atplt = autopilot()
	mvc = mavcon()
	mvc.setarm(1)
	time.sleep(2)
	mvc.offboard()
	mvc.gotopose(0.0, 0.0, 3.275)
	time.sleep(5)
	mvc.gotopose(2.0, 0.0, 3.275)
	mvc.gotopose(2.5, 0.0, 3.275)
	time.sleep(5)
	# mvc.gotopose(2.5, 1.0, 3.275)
	# time.sleep(5)
	# mvc.gotopose(2.5, 0.0, 3.275)
	# mvc.gotopose(2.5, -1.0, 3.275)
	# time.sleep(5)
	# mvc.gotopose(2.5, 0.0, 3.275)


	i = 0
	while i < 15:
		atplt.explore()
		atplt.getFrameNum()
		mvc.gotopose(atplt.wplist[i].x - 0.5, atplt.wplist[i].y, 3.275)
		mvc.gotopose(atplt.wplist[i].x, atplt.wplist[i].y, 3.275)
		print("Passed through frame " + str(atplt.currentframe+1))
		print('--------------------------------------------')
		atplt.incrementCurrentFrame()
		mvc.gotopose(atplt.wplist[i].x + 0.5, atplt.wplist[i].y, 3.275)
		if i<14:
			print("Moving to next frame " + str(atplt.currentframe+2))
			print("Coordinates: ")
			print(str(atplt.wplist[i+1]))
		time.sleep(3)
		atplt.currentframe += 1
		i += 1

	print("Heading towards landing zone")
	mvc.gotopose(20.0,0.0,0.1)
	maintimeend = time.time()
	print("Total Time in Minutes = " + str(round((maintimeend-maintimestart)/60,4)))
	print("MISSION COMPLETED")
	print("Successfully passed throught all 15 Frames")
	print("TEAM UMIC AERIAL")
	rospy.spin()


if __name__ == '__main__':
	main()
