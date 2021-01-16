#!/usr/bin/env python
import rospy
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
import tf, geometry_msgs, tf2_ros
from tf import TransformBroadcaster
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

def callback(value):
	global pc_arr
	pc_arr = ros_numpy.numpify(value)

def display(msg):
	bridge2 = CvBridge()
	depth_img = bridge2.imgmsg_to_cv2(msg,desired_encoding='passthrough')
	cv2.namedWindow('depth_img',cv2.WINDOW_NORMAL)
	cv2.imshow('depth_img',depth_img)

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

def pixel_to_depth(h,w,arr):
	xp,yp,zp = arr['x'][h][w],arr['y'][h][w],arr['z'][h][w]
	#cord = np.array([xp,yp,zp])
	ps = PointStamped()
	ps.header.frame_id = "r200link"
	ps.header.stamp = rospy.Time(0)
	ps.point.x = zp
	ps.point.y = -xp
	ps.point.z = -yp
	mat = listener.transformPoint("/map", ps)
	#print(yp,mat.point.y)
	return mat

def cam_frame(data):
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
			#imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
			if (width<630 and (width/height < 3.0) and (height/width < 1.0) and (box[0][1] < 476) and (box[3][1] < 476)):
				imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
				a1,b1 = box[0][0],box[0][1]
				a2,b2 = box[1][0],box[1][1]
				a3,b3 = box[2][0],box[2][1]
				a4,b4 = box[3][0],box[3][1]
				cloud_arr = pc_arr

				mat1 = pixel_to_depth(b1,a1,cloud_arr)
				mat2 = pixel_to_depth(b2,a2,cloud_arr)
				mat3 = pixel_to_depth(b3,a3,cloud_arr)
				mat4 = pixel_to_depth(b4,a4,cloud_arr)
				
				fx=(mat1.point.x+mat2.point.x+mat3.point.x+mat4.point.x)/4
				fy=(mat1.point.y+mat2.point.y+mat3.point.y+mat4.point.y)/4
				fz=(mat1.point.z+mat2.point.z+mat3.point.z+mat4.point.z)/4
				#print(mat4,b4,a4)
				#print(mat)
				mat = Point()
				mat.x = fx
				mat.y = fy
				mat.z = fz
				
				if(0.8 < np.abs(mat2.point.y - mat3.point.y) < 1.2):
					if (np.abs(mat1.point.x-mat2.point.x)<0.3 and np.abs(mat2.point.x-mat3.point.x)<0.3 and np.abs(mat3.point.x-mat4.point.x)<0.3 and np.abs(mat1.point.x-mat4.point.x)<0.3):
						if(2.8<fz<3.5):
							#print(mat)
							#imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
							cv2.circle(img, (a1,b1),5, (255,0,0), 2)
							cv2.circle(img, (a2,b2),5, (255,0,0), 2)
							cv2.circle(img, (a3,b3),5, (255,0,0), 2)
							cv2.circle(img, (a4,b4),5, (255,0,0), 2)
							cv2.putText(img, 'x= {}, y= {}, z = {}'.format(round(fx,2),round(fy,2),round(fz,2)), (a1,b1), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 255, 0), 1)
							if(3.8<fx<4.2):
								col[0] = mat
							elif(4.8<fx<5.2):
								col[1] = mat
							elif(5.8<fx<6.2):
								col[2] = mat
							elif(6.8<fx<7.2):
								col[3] = mat
							elif(7.8<fx<8.2):
								col[4] = mat
							elif(8.8<fx<9.2):
								col[5] = mat
							elif(9.8<fx<10.2):
								col[6] = mat
							elif(10.8<fx<11.2):
								col[7] = mat
							elif(11.8<fx<12.2):
								col[8] = mat
							elif(12.8<fx<13.2):
								col[9] = mat
							elif(13.8<fx<14.2):
								col[10] = mat
							elif(14.8<fx<15.2):
								col[11] = mat
							elif(15.8<fx<16.2):
								col[12] = mat
							elif(16.8<fx<17.2):
								col[13] = mat
							elif(17.8<fx<18.2):
								col[14] = mat
							#print(col,'dfsdjfksbdfhjsbfsdjk')
							frame_list.centers = col
	cv2.namedWindow('detected_image',cv2.WINDOW_NORMAL)
	# cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
	# cv2.imshow('mask',mask)
	cv2.imshow('detected_image',img)
	cv2.waitKey(30)
	pub.publish(frame_list)


if __name__ == '__main__':
	rospy.init_node('world_coordinate', anonymous=True)
	print('node_initialised')
	global frame_list
	global col
	frame_list = frame()
	point = Point()
	point.x = 0.0
	point.y = 0.0
	point.z = 0.0
	col = [point]*15
	frame_list.centers = col
	listener = tf.TransformListener()
	rospy.Subscriber("/r200/color/image_raw", Image, cam_frame)
	rospy.Subscriber("/r200/depth/points", PointCloud2, callback)
	#rospy.Subscriber("/r200/depth/image_raw", Image, display)
	pub = rospy.Publisher('frame_centers/position', frame, queue_size=100)
	rospy.spin()