#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d
from gazebo_msgs.msg import ModelStates

def callback(data):
    global count, x, y, z
    a = data.name
    c = 0
    for i in a:
        if (i=='iris'):
            b = c
        c+=1
    #print(b)
    p = data.pose[b]
    x.append(p.position.x)
    y.append(p.position.y)
    z.append(p.position.z)

def orbcall(val):
    global ox, oy, oz
    ox.append(val.pose.position.x)
    oy.append(val.pose.position.y)
    oz.append(val.pose.position.z)

def listener():
    rospy.init_node('plotpath', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.Subscriber("/orb_slam2_rgbd/pose", PoseStamped, orbcall)

if __name__ == '__main__':
    x=[]
    y=[]
    z=[]
    ox=[]
    oy=[]
    oz=[]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.axis('equal')
    listener()
    def animate(i):
        plt.cla()
        ax.plot3D(x, y, z, 'g', label = 'SLAM Estimated Pose')
        ax.plot3D(ox, oy, oz, 'r', label = 'Actual Pose')
        ax.legend()
    ani = FuncAnimation(plt.gcf(), animate, interval=100)
    plt.tight_layout()
    plt.show()
