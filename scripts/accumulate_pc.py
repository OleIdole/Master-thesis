#!/usr/bin/env python
# accumulate_pc.py

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
import pcl_ros
import pcl
#importlib.util.spec_from_file_location("module.name", "/path/to/file.py")
import pcl_conversions
#import pcl_conversions
#import point_cloud
#import point_types

# Check code example at: http://wiki.ros.org/pcl/Tutorials/hydro?action=AttachFile&do=view&target=example_voxelgrid.cpp

class AccumulatePC():
    def __init__(self):
        self.pcPub = rospy.Publisher("/accumulatedPointCloud", pc2, queue_size=1)
        self.laserPcSub = rospy.Subscriber("/laserPointCloud", pc2, self.pointCloudCallback)

    def pointCloudCallback(self, data):
        cloud = pc2()
        self.pcPub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node("accumulatePointCloud")
    accPC = AccumulatePC()
    rospy.spin()