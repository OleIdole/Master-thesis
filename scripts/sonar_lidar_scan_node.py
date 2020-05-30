#!/usr/bin/env python
# lidar_restrictions.py

import rospy
import math
from sensor_msgs.msg import LaserScan
from math import *
import numpy as np
import copy
from my_package.msg import LaserScanFiltered
from turtlebot3_msgs.msg import SensorState

class sonar_lidar_scan_node:
    def __init__(self):
        # Defining variables:
        self.scanFiltered = LaserScanFiltered()
        self.scanFiltered.laser_range = 0.0
        self.scanFiltered.laser_intensity = 0.0
        self.scanFiltered.sonar_range = 0.0
        self.sensor_offset = 13.5

        rospy.Subscriber("/scan", LaserScan, self.callback_get_lidar_scan)
        rospy.Subscriber("/sensor_state", SensorState, self.callback_get_sonar_scan)
        self.pub = rospy.Publisher("/sonar_lidar_state", LaserScanFiltered, queue_size=10)
        rospy.spin()

    def callback_get_lidar_scan(self, data):
        # Convert data to lists
        ranges_filter = list(data.ranges)
        intensities_filter = list(data.intensities)

        # Extract front center scan data
        self.scanFiltered.laser_range = ranges_filter[0] * 100
        self.scanFiltered.laser_intensity = intensities_filter[0]

        # Publish new filtered scan
        self.publish()

    def callback_get_sonar_scan(self, data):
        self.scanFiltered.sonar_range = data.sonar

    def publish(self):
        # Calculate sensor deviation and publish filtered scan
        self.scanFiltered.deviation = self.scanFiltered.laser_range - self.scanFiltered.sonar_range - self.sensor_offset
        self.pub.publish(self.scanFiltered)

    def handle_offset(self):
        if self.scanFiltered.sonar_range < 6.0:
            self.sensor_offset = 12
        elif self.scanFiltered.sonar_range < 20.0:
            self.sensor_offset = 14.0
        elif self.scanFiltered.sonar_range < 40.0:
            self.sensor_offset = 16.0
        else:
            self.sensor_offset = 20.0


if __name__ == '__main__':
	try:
		rospy.init_node('sonar_lidar_scan')
		sn = sonar_lidar_scan_node()
	except rospy.ROSInterruptException:
		pass