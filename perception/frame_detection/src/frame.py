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
import tf, geometry_msgs, tf2_ros
from tf import TransformBroadcaster
#import sensor_msgs.point_cloud2
#import sensor_msgs.point_cloud2 as pc2
#from sensor_msgs.msg import PointCloud2

def callback(value):
	global val
	val = value

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

def pixel_to_depth(h,w,depth_array):
	if(h<480 and w<640):
		f = 381.362467
		ipp = np.array([[w],[h],[1]]) # 2d target coord 
		b = np.array([[381.362467, 0.0, 320.5], [0.0, 381.362467, 240.5], [0.0, 0.0, 1]])
		invk = np.linalg.inv(b)
		rf = np.array([[0, 0, f], [-f, 0, 0], [0, -f, 0]]) # focal lenght matrix
		invf = np.dot(rf, invk)
		ipt = np.dot(invf,ipp) # 3d image plane target cood
		(trans,rot) = lisn.lookupTransform("/world", "r200link", rospy.Time(0))
		euler = tf.transformations.euler_from_quaternion(rot)
		rm = tf.transformations.euler_matrix(euler[0],euler[1],euler[2])
		wfrm = [[rm[0][0],rm[0][1],rm[0][2]],[rm[1][0],rm[1][1],rm[1][2]],[rm[2][0],rm[2][1],rm[2][2]]] # Transformation matrix for world frame
		detipt = sqrt(ipt[0]**2 + ipt[1]**2 + ipt[2]**2) # magnitude of ipt
		uipt = ipt/detipt
		ls = np.array(np.dot(wfrm,uipt))
		lr = np.array([1,0,0])
		cos = np.dot(lr,ls)
		h = depth_array[h,w]
		d = h/cos
		cord = d*(uipt) # 3d target coord in camera frame
		#print(cord, 'in cam frame')
		#print(h)
		ps = PointStamped()
		ps.header.frame_id = "r200link"
		ps.header.stamp = rospy.Time(0)
		ps.point.x = cord[0]
		ps.point.y = cord[1]
		ps.point.z = cord[2]
		mat = listener.transformPoint("/world", ps)
		return mat


def cam_frame(data):
	#rate = rospy.Rate(10)
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
			if (width<630 and (width/height < 3) and (height/width < 1.5) and (box[0][1] < 476) and (box[3][1] < 476)):
				imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
				a1,b1 = box[0][0]-1,box[0][1]-1
				a2,b2 = box[1][0]-1,box[1][1]+1
				a3,b3 = box[2][0]+1,box[2][1]+1
				a4,b4 = box[3][0]+1,box[3][1]-1

				depth = val
				bridged = CvBridge()
				depth_image =bridged.imgmsg_to_cv2(depth,desired_encoding='passthrough')
				depth_array = np.array(depth_image,dtype=np.dtype(np.float32))
				mat1 = pixel_to_depth(b1,a1,depth_array)
				mat2 = pixel_to_depth(b2,a2,depth_array)
				mat3 = pixel_to_depth(b3,a3,depth_array)
				mat4 = pixel_to_depth(b4,a4,depth_array)

				fx=(mat1.point.x+mat2.point.x+mat3.point.x+mat4.point.x)/4
				fy=(mat1.point.y+mat2.point.y+mat3.point.y+mat4.point.y)/4
				fz=(mat1.point.z+mat2.point.z+mat3.point.z+mat4.point.z)/4
				loc = Point()
				loc.x = fx
				loc.y = fy
				loc.z = fz
				#print(mat1[2],mat2[2],mat3[2],mat4[2])
				#print(mat4)
				if (np.abs(mat1.point.z-mat2.point.z)<0.3 and np.abs(mat2.point.z-mat3.point.z)<0.3 and np.abs(mat3.point.z-mat4.point.z)<0.3 and np.abs(mat1.point.z-mat4.point.z)<0.3):
					#print(loc)
					if(3.0<fz<3.5):
						if(3.9<fx<4.1):
							col[0] = loc
						elif(4.9<fx<5.1):
							col[1] = loc
						elif(5.9<fx<6.1):
							col[2] = loc
						elif(6.9<fx<7.1):
							col[3] = loc
						elif(7.9<fx<8.1):
							col[4] = loc
						elif(8.9<fx<9.1):
							col[5] = loc
						elif(9.9<fx<10.1):
							col[6] = loc
						elif(10.9<fx<11.1):
							col[7] = loc
						elif(11.9<fx<12.1):
							col[8] = loc
						elif(12.9<fx<13.1):
							col[9] = loc
						elif(13.9<fx<14.1):
							col[10] = loc
						elif(14.9<fx<15.1):
							col[11] = loc
						elif(15.9<fx<16.1):
							col[12] = loc
						elif(16.9<fx<17.1):
							col[13] = loc
						elif(17.9<fx<18.1):
							col[14] = loc
						print(col,'dfsdjfksbdfhjsbfsdjk')
						frame_list.centers = col


	cv2.imshow('image',img)
	cv2.waitKey(30)
	pub.publish(frame_list)
	#rate.sleep()


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
	lisn = tf.TransformListener()
	listener = tf.TransformListener()
	rospy.Subscriber("/r200/color/image_raw", Image, cam_frame)
	rospy.Subscriber("/r200/depth/image_raw", Image, callback)
	pub = rospy.Publisher('frame_center/position', frame, queue_size=100)
	rospy.spin()