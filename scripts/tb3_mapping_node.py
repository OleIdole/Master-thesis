#!/usr/bin/env python
# tb3_mapping_node.py

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

class tb3_mapping_node():
    def __init__(self):
        
        # Variables
        self.laserProj = LaserProjection()

        # Publishers
        self.pcWorldPub = rospy.Publisher("/world_pc", pc2, queue_size=1)
        #self.pcWaterPub = rospy.Publisher("/water_pc", pc2, queue_size=1)
        self.mkArrayPub = rospy.Publisher("/marker_array_water_recoloured", MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.callback_lidar_scan)
        rospy.Subscriber("/marker_array_water", MarkerArray, self.callback_recolour)

        rospy.spin()

    # Publish pc2 of world for the 1st Octomap server to use for mapping.
    def callback_lidar_scan(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        self.pcWorldPub.publish(cloud_out)

    # Listen to markerarray from 2nd Octomap server and modify to distinguish water from the world.
    def callback_recolour(self, data):
            tempMarker = data.markers[len(list(data.markers))-1] # Only final marker element contains the markers.
            tempMarker.header.stamp = rospy.Time() # Do not set Time.now() here, it can cause issues for marker being accepted.
            tempMarker.color.r = 0.0
            tempMarker.color.g = 0.9167
            tempMarker.color.b = 1.0
            tempMarker.color.a = 1.0
            tempMarker.scale.z = 0.051 # Slightly larger than map resolution to make water voxels cover world voxels of same location.
            tempMarker.colors.append(tempMarker.color)
            self.mkArrayPub.publish([tempMarker]) # Publish the marker as a MarkerArray

if __name__ == '__main__':
    rospy.init_node("tb3_mapping_node")
    node = tb3_mapping_node()
    rospy.spin()