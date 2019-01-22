#!/usr/bin/env python
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import Point

from sensor_msgs.msg import LaserScan

import math

# pt=Point()
# pt.x=0
# pt.y=0
# pt.z=0

def callback(corddata):

	#print ("Angle: %s", corddata.x)
	
	pt.x=corddata.x
	pt.y=0
	pt.z=0
	#print ("Angle: %s", corddata.x)

	# rate=rospy.Rate(5)
	# pub.publish(pt)
	# rate.sleep()


def LIDAR(ldata):
	#print ("Distance to lidar: ", ldata.ranges[180])

	#print ("Distance to lidar: ", ldata.ranges[0])
	global pt2

	pt2=Point()

	min_lid = 0.12
	max_lid = 3.5

	# minDist = min(list(filter(lambda x: x<max_lid and x>min_lid, ldata.ranges)))
	minDist = float("inf")
	minAngle = 0
	for (angle, dist) in enumerate(ldata.ranges):
		if dist > min_lid and dist < max_lid and (angle>270 or angle<90):
			if dist < minDist:
				minDist = dist
				minAngle = angle

	if minAngle >270 and minAngle < 360:
		minAngle = minAngle - 360

	minAngle = (math.pi)/180 * minAngle


	pt2.x = minDist
	pt2.y = minAngle

	pub2.publish(pt2)

	#print ("Ranges:",minDist)



	#print("Size:",ldata.ranges[0:90])

	#print ("pt.x",pt.x)

	# A = ldata.ranges[270:360]

	# A=np.asarray(A)

	# B=np.nonzero(A)

	# B=np.asarray(B)

	# B.astype(int)

	# A = A[B]

	# if pt.x < 0: 
	# 	C = ldata.ranges[310:360]

	# 	D = ldata.ranges[0:30]

	# 	A = C + D

	# 	A=np.asarray(A)

	# 	B=np.nonzero(A)

	# 	B=np.asarray(B)

	# 	B.astype(int)

	# 	A = A[B]

	# 	minval = np.amin(A)


	# else:

	# 	C = ldata.ranges[330:360]

	# 	D = ldata.ranges[0:50]

	# 	A = C+D

	# 	A=np.asarray(A)

	# 	B=np.nonzero(A)

	# 	B=np.asarray(B)

	# 	B.astype(int)

	# 	A = A[B]

	# 	minval = np.amin(A)

	# #print ("min value:",minval)

	# pt2.x=minval
	# pt2.y=0
	# pt2.z=0

	# rate=rospy.Rate(5)
	# pub2.publish(pt2)
	# rate.sleep()

	#print ("Ranges:",pt2.x)


def init():

	global pub
	global pub2

	global pt
	pt=Point()

	pt.x=0
	pt.y=0
	pt.z=0

	rospy.init_node('get_object_range', anonymous=True)

	#rospy.Subscriber("/imageLocation", Point, callback)

	pub2 = rospy.Publisher('lin_cord', Point, queue_size=3)

	rospy.Subscriber("/scan",LaserScan, LIDAR)

	rospy.spin()


if __name__ == '__main__':
	try:
		init()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"

# rate = rospy.Rate(5)


# while not rospy.is_shutdown():

# 	print ("x:",pt.x)
# 	print ("y:",pt.y)
# 	pub.publish(pt)
# 	rate.sleep()







