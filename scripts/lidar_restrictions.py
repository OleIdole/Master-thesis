#!/usr/bin/env python
# lidar_restrictions.py

import rospy
import math
from sensor_msgs.msg import LaserScan
from math import *
import numpy as np
import copy
from master-thesis.msg import LaserScanFiltered

#ranges_filter = []
#intensities_filter = []
laser_range = 0.0
laser_intensity = 0.0

#copy the range and intensities of "/scan" topic to "ranges_filter" and "intensities_filter" 
#you need to convert them to "list" as "data.ranges" and "data.intensities" are "tuple"
def callback_scan(data):
    global laser_range, laser_intensity # ranges_filter, intensities_filter

    len(data.ranges) #360
    len(data.intensities) #360

    ranges_filter = copy.copy(data.ranges)
    intensities_filter = copy.copy(data.intensities)

    #convert them to list
    ranges_filter = list(ranges_filter)
    intensities_filter = list(intensities_filter)

     #filtering those angles that I do not want them (based on the question)
    #for x in range(1, 360):
    #    ranges_filter[x] = 0
    #    intensities_filter[x] = 0

    laser_range = ranges_filter[0]
    laser_intensity = intensities_filter[0]

#define a new topic called "filterScan" to store all laser scanner data
rospy.init_node('laser_scan_filter')

scan_pub = rospy.Publisher('filterScan', LaserScanFiltered, queue_size=10)

rospy.Subscriber("/scan", LaserScan, callback_scan) 

#it is based on the type of laser scanner (length of data.ranges)
num_readings = 360
laser_frequency = 60

count = 0
r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()



    filterScan = LaserScanFiltered()
    filterScan.range = laser_range
    filterScan.intensity = laser_intensity

    scan_pub.publish(filterScan)
    r.sleep()