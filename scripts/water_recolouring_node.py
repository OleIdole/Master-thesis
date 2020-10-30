#!/usr/bin/env python
# water_recolouring_node.py

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, Point

class water_recolouring_node():
    def __init__(self):
        self.mkArrayPub = rospy.Publisher("/marker_array_water_recoloured", MarkerArray, queue_size=1)
        rospy.Subscriber("/marker_array_water", MarkerArray, self.callback_recolour)

        rospy.spin()

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

if __name__ == '__main__':
    rospy.init_node("water_recolouring_node")
    node = water_recolouring_node()
    rospy.spin()