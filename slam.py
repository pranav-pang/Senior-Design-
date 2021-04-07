#!/usr/bin/python
import numpy as np
import math 
import rospy
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

points_x = []
points_y = []
points_z = []

def hook():
	print("here")
	with open('listx.txt', 'w') as filehandlex:
		for x in points_x:
			filehandlex.write("{:.4f}\n".format(x))

	with open('listy.txt', 'w') as filehandley:
		for y in points_y:
			filehandley.write("{:.4f}\n".format(y))
	with open('listz.txt', 'w') as filehandlez:
		for z in points_z:
			filehandlez.write("{:.4f}\n".format(z))
	#fig = plt.figure()
	#ax = plt.axes(projection='3d')
	#ax.plot3D(np.array(points_x), np.array(points_y), np.array(points_z))

def update_line(new_data):
	x = new_data.position.x
	y = new_data.position.y
	z = new_data.position.z

	print(type(x))

	if (x == 0 or y == 0 or z == 0):
		x = last_x
		y = last_y
		z = last_z
	else:	
		last_x = x
		last_y = y
		last_z = z

	points_x.append(x)
	points_y.append(y)
	points_z.append(z)

def callback(data):
	print(data.pose.pose)
	update_line(data.pose.pose)

def listener():
	rospy.init_node('slam_listener', anonymous=True)

	rospy.Subscriber('/rtabmap/odom', Odometry, callback)
	rospy.on_shutdown(hook)
	rospy.spin() 

if __name__ == '__main__':
    	listener()
