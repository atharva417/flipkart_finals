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
from geometry_msgs.msg import PointStamped, PoseStamped

def scale_contour(cnt, scale):
	M = cv2.moments(cnt)
	if M['m00']==0:
		return cnt
	else:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cnt_norm = cnt - [cx, cy]
		cnt_scaled = cnt_norm * scale
		cnt_scaled = cnt_scaled + [cx, cy]
		cnt_scaled = cnt_scaled.astype(np.int32)
		return cnt_scaled

vidcap = cv2.VideoCapture('/home/atharva/localisation/output.avi')
success,frame = vidcap.read()
while success:
	success,frame = vidcap.read()

#frame = cv2.imread('/home/atharva/localisation/test3.jpg')

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	l_h = 20 #max_value_H*0.1
	l_s = 70 #max_value*0.3
	l_v = 70 #max_value*0.35
	u_h = 50 #max_value_H*0.25
	u_s = 255
	u_v = 255

	lower_red = np.array([l_h, l_s, l_v])
	upper_red = np.array([u_h, u_s, u_v])

	mask = cv2.inRange(hsv, lower_red, upper_red)
	kernel = np.ones((3,3), np.uint8)
	mask = cv2.dilate(mask, kernel)
	kernel2 = np.ones((4,4), np.uint8)
	mask = cv2.erode(mask, kernel2)

	if int(cv2.__version__[0]) > 3:
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for cnt in contours:
		cnt = scale_contour(cnt, 1.01)
		#approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(cnt)
		if (150000 > area > 2000):
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			width = box[1][0]-box[2][0]
			height = box[0][1]-box[1][1]
			if (width<630 and (width/height < 3) and (height/width < 1.5) and (box[0][1] < 476) and (box[3][1] < 476)):
				img = cv2.drawContours(frame, [box], 0, (0,0,255), 2)

	cv2.imshow('image',frame)
	cv2.imshow('mask',mask)
	cv2.waitKey(30)

cv2.destroyAllWindows()