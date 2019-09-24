#!/usr/bin/env python
# Example 7.2 range_ahead.py pg. 105 Programming Robots with ROS
# modified by R. Kube, Jan. 21, 2019
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # print the range/distance of the object directly ahead in metres
	range_ahead = msg.ranges[len(msg.ranges)/2]
	print "range ahead: %0.1f" % range_ahead

def distance_ahead():
    # this node subscribes to the /scan topic and calls the 'scan_callback'
    # function for every published message
    rospy.init_node('range_ahead')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()    # loop continuously

if __name__ == '__main__':
    distance_ahead()
