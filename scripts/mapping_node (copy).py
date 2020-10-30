#!/usr/bin/env python
# mapping_node.py

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan, Range
from laser_geometry import LaserProjection
import math
from math import *
import numpy as np
import copy
from master_thesis.msg import LaserScanFiltered
from std_srvs.srv import SetBool, SetBoolResponse
#from std_msgs.msg import UInt16

class mapping_node():
    def __init__(self):
        self.isWater = False

        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.callback_lidar_scan)
        self.service_set_isWater = rospy.Service('is_water', SetBool , self.callback_isWater)           # Service this node offers

        rospy.spin()

    def callback_lidar_scan(self, data):
        # Only act if water is classified
        if(self.isWater==True):
            # Keep forward range data of Laserscan, discard rest
            scan = data
            scan.ranges = [data.ranges[0]]
            scan.intensities = [data.intensities[0]]
            
            # Convert to PointCloud2
            cloud_out = self.laserProj.projectLaser(scan)

            # Publish PointCloud2
            self.pcPub.publish(cloud_out)

            # Reset flag until next positive water classification
            self.isWater = False

    def callback_isWater(self, is_water):
        if is_water.data==True:
            self.isWater = True
            return SetBoolResponse(True, 'Mapping water')
        elif is_water.data==False:
            self.isWater = False
            return SetBoolResponse(True, 'No longer mapping water')

if __name__ == '__main__':
    rospy.init_node("mapping_node")
    node = mapping_node()
    rospy.spin()