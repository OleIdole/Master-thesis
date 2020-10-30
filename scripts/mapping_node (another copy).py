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
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, Point

class mapping_node():
    def __init__(self):
        
        # Variables
        self.laserProj = LaserProjection()
        self.isWater = False

        # Publishers
        self.pcWorldPub = rospy.Publisher("/world_pc", pc2, queue_size=1)
        self.pcWaterPub = rospy.Publisher("/water_pc", pc2, queue_size=1)
        self.mkArrayPub = rospy.Publisher("/marker_array_water_recoloured", MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.callback_lidar_scan)
        rospy.Subscriber("/marker_array_water", MarkerArray, self.callback_recolour)

        # Services
        self.service_set_isWater = rospy.Service('is_water', SetBool , self.callback_isWater) # Toggle for isWater when water is classified

        rospy.spin()

    def callback_lidar_scan(self, data):
        # Mapping behaviour when water found.
        if(self.isWater==True):
            # Keep forward range data of Laserscan, discard rest
            frontScan = copy.deepcopy(data)
            frontScan.ranges = list(frontScan.ranges)
            frontScan.intensities = list(frontScan.intensities)

            # Remove measurements that are not in front, delete more points for higher precision of water
            # Using more than just the value straight in front because the water is likely spreading a bit
            for i in range (20,340,1):
                frontScan.ranges[i] = -1
                frontScan.intensities[i] = -1
            
            # Convert to PointCloud2
            water_pc = self.laserProj.projectLaser(frontScan)

            # Keep everything except forward range data of Laserscan
            fullScan = copy.deepcopy(data)
            fullScan.ranges = list(fullScan.ranges)
            fullScan.intensities = list(fullScan.intensities)

            for i in range (340,360,1):
                fullScan.ranges[i] = -1
                fullScan.intensities[i] = -1

            for i in range (0,20,1):
                fullScan.ranges[i] = -1
                fullScan.intensities[i] = -1

            # Convert to PointCloud2
            cloud_out = self.laserProj.projectLaser(fullScan)

            # Publish PointCloud2
            self.pcWaterPub.publish(water_pc)
            self.pcWorldPub.publish(cloud_out)

            # Reset flag until next positive water classification
            #self.isWater = False

        # Mapping behaviour when no water found.
        else:
            cloud_out = self.laserProj.projectLaser(data)
            self.pcWorldPub.publish(cloud_out)

    def callback_recolour(self, data):
            tempMarker = data.markers[len(list(data.markers))-1] # Only final marker element contains the markers.
            tempMarker.header.stamp = rospy.Time() # Do not set Time.now() here, it can cause issues for marker being accepted.
            tempMarker.color.r = 0.0
            tempMarker.color.g = 0.9167
            tempMarker.color.b = 1.0
            tempMarker.color.a = 1.0
            tempMarker.scale.z = 0.051
            tempMarker.colors.append(tempMarker.color)

            self.mkArrayPub.publish([tempMarker]) # Publish the marker as a MarkerArray

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