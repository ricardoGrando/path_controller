#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math
from gazebo_msgs.msg import *
import numpy as np
import csv
import rospkg
import time

laser = LaserScan()
pose = Pose()
rospack = rospkg.RosPack()
_map = np.array(list(csv.reader(open(rospack.get_path('path_controller')+"/scripts/"+"map.csv", "r"), delimiter=" "))).astype("int")
map_resolution = 4

def path_callback(data):
	global pose
	pose = data.pose[-1]

	index_x = int(pose.position.x*map_resolution)
	index_y = int(pose.position.y*map_resolution)

	if (index_x < 1): index_x = 1
	if (index_x > _map.shape[0]-1): index_x = _map.shape[0]-1
	if (index_y < 1): index_y = 1
	if (index_y > _map.shape[0]-1): index_y = _map.shape[0]-1

	if (_map[_map.shape[0] - index_y][index_x] == 1):
		_map[_map.shape[0] - index_y][index_x] = 2

		rospy.loginfo("Another part aspirated ... percentage total aspirated.... %s ", 100*float(np.count_nonzero(_map == 2))/(np.count_nonzero(_map == 1) + np.count_nonzero(_map == 2)) )
		rospy.loginfo("Discrete Map")
		rospy.loginfo("%s", str(_map))

def laser_callback(data):
	global laser
	laser = data

if __name__ == "__main__":
	rospy.init_node("path_controller_node", anonymous=False)

	rospy.Subscriber("/gazebo/model_states", ModelStates, path_callback)

	rospy.Subscriber("/scan", LaserScan, laser_callback)

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(5) # 10hz
	velocity = Twist()
	while not rospy.is_shutdown():
		# FACA SEU CODIGO AQUI
		if (len(laser.ranges) > 0):
			if (min(laser.ranges[90:270]) > 0.25):
				velocity.linear.x = random.uniform(-0.1, -0.25)
				velocity.angular.z = 0.0
			else:
				velocity.linear.x = 0.0
				velocity.angular.z = 0.25

			pub.publish(velocity)

		r.sleep()

# DISCRETE MAP
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
# 0 1 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 0
# 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 1 0
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
