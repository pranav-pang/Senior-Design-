#!/usr/bin/env python 

import numpy as np
import math 
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Quaternion

def callback(data):
    rospy.loginfo(data.data)

def listener():
    rospy.init_node('slam_listener', anonymous=True)

    rospy.Subscriber('/rtabmap/odom/pose/pose')

    rospy.spin()

if __name__ == '__main__':
    listener()