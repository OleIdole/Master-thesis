#!/usr/bin/env python
# tb3_classification_node.py

import rospy
import math
from sensor_msgs.msg import LaserScan, Range
from math import *
import numpy as np
import copy
from master_thesis.msg import LaserScanFiltered
from turtlebot3_msgs.msg import SensorState

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection

class tb3_classification_node:
    def __init__(self):
        # Defining variables:
        self.laserProj = LaserProjection()

        self.scanFiltered = LaserScanFiltered()
        self.scanFiltered.laser_range = 0.0
        self.scanFiltered.laser_intensity = 0.0
        self.scanFiltered.sonar_range = 0.0
        self.sensor_offset = 10.0

        rospy.Subscriber("/scan", LaserScan, self.callback_get_lidar_scan)
        rospy.Subscriber("/sensor_state", SensorState, self.callback_get_sonar_scan)
        self.pub = rospy.Publisher("/sonar_lidar_state", LaserScanFiltered, queue_size=10)
        self.pcWaterPub = rospy.Publisher("/water_pc", pc2, queue_size=1)
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

        if(self.scanFiltered.is_water == True):
            # Construct LaserScan message
            sonarScanMsg = LaserScan()
            sonarScanMsg.header = data.header
            sonarScanMsg.header.frame_id = "base_scan"
            sonarScanMsg.angle_min = -0.1
            sonarScanMsg.angle_max = 0.1
            sonarScanMsg.angle_increment = 0.1
            sonarScanMsg.range_min = 0.02 # unit in meters
            sonarScanMsg.range_max = 2.0 # unit in meters
            sonarDist = data.sonar /100 # Convert from cm to meters
            sonarScanMsg.ranges = [sonarDist, sonarDist, sonarDist] # Easy fix because not able to make pointcloud with 1 point.

            # Convert to PointCloud2
            cloud_out = self.laserProj.projectLaser(sonarScanMsg)

            # Publish PointCloud2
            self.pcWaterPub.publish(cloud_out)

    def publish(self):
        # Calculate sensor deviation and publish filtered scan
        self.scanFiltered.deviation = self.scanFiltered.laser_range - self.scanFiltered.sonar_range - self.sensor_offset

        if(self.scanFiltered.deviation >= 3.0 and self.scanFiltered.laser_intensity >= 4100 and self.scanFiltered.sonar_range != 0):
            self.scanFiltered.is_water = True
        else:
            self.scanFiltered.is_water = False

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
		rospy.init_node('tb3_classification_node')
		sn = tb3_classification_node()
	except rospy.ROSInterruptException:
		pass