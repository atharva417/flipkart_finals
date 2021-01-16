#!/usr/bin/env python
import rospy
#import sys
from pylab import *
import numpy as np
import time
from matplotlib import pyplot as plt
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from frame_detection.msg import frame

if __name__ == '__main__':
	rospy.init_node('world_coordinate', anonymous=True)
	frame_list = frame()
	point = Point()
	point.x = 0.0
	point.y = 0.0
	point.z = 0.0
	col = [point]*15
	frame_list.centers = col
	pub = rospy.Publisher('frame_center/position', frame, queue_size=100)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rospy.loginfo(point)
		pub.publish(frame_list)
		rate.sleep()