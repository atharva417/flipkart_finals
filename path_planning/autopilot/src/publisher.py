#!/usr/bin/env python
# ROS python API
import rospy
import time
import os
import rospy
from frame_detection.msg import frame
from geometry_msgs.msg import PointStamped, PoseStamped, Point


def main():
	rospy.init_node('publisher')
	pubnew = rospy.Publisher('/frame_center/position', frame, queue_size = 10)
	frames = frame()
	wplist = []
	col = [[4.0, 1.0, 3.22],[5.0, -1.5, 3.22], [6.0, 1.5, 3.22], [7.0 ,-0.8 ,3.22], [8.0, 1.0 ,3.22], [9, -1.5 ,3.22], [10.0, 1.5, 3.22], [11.0, -1.0, 3.22], [12.0, 1.0, 3.22], [13.0 , -0.8 ,3.22], [14.0 ,1.0, 3.22], [15.0, -1.5 ,3.22], [16.0, 1.4 ,3.22], [17.0, -1.0, 3.22], [18.0 ,1.0, 3.22]]
	for wp in col:
		point = Point()
		point.x = wp[0]
		point.y = wp[1]
		point.z = wp[2]
		wplist.append(point)

	#print(wplist)
	frames.centers = wplist
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pubnew.publish(frames)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass