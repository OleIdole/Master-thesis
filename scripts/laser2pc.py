#!/usr/bin/env python
# laser2pc.py

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
#import pcl_ros
#import pcl_conversions

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        #self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        #print(len(data.ranges))
        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()