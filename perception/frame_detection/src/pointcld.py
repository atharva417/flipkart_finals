#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelStates
import tf2_ros
import tf

def handle_pose(msg):
    st = tf2_ros.StaticTransformBroadcaster()
    br = tf.TransformBroadcaster()

    tf2Stamp = TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = "base_link"
    tf2Stamp.child_frame_id = "camera_link"
    tf2Stamp.transform.translation.x = 0.1
    tf2Stamp.transform.translation.y = 0.0
    tf2Stamp.transform.translation.z = 0.0

    #quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

    tf2Stamp.transform.rotation.x = -0.5
    tf2Stamp.transform.rotation.y = -0.5
    tf2Stamp.transform.rotation.z = 0.5
    tf2Stamp.transform.rotation.w = 0.5

    st.sendTransform(tf2Stamp)
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "/map")


if __name__ == '__main__':
    rospy.init_node('cam_tf_broadcaster')
    rospy.Subscriber('/mavros/local_position/pose',
                     PoseStamped,
                     handle_pose)
    print('Broadcasting...')
    rospy.spin()