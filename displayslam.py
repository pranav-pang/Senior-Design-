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

with open('listx.txt', 'r') as filehandlex:
	for line in filehandlex:
		currentPlace = line[:-1]
		points_x.append(float(currentPlace))

with open('listy.txt', 'r') as filehandley:
	for line in filehandley:
		currentPlace = line[:-1]
		points_y.append(float(currentPlace))

with open('listz.txt', 'r') as filehandlez:
	for line in filehandlez:
		currentPlace = line[:-1]
		points_z.append(float(currentPlace))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(0, 20)
ax.set_ylim3d(0, 20)
ax.set_zlim3d(0, 20)
ax.plot3D(np.array(points_x), np.array(points_y), np.array(points_z))
plt.show()
