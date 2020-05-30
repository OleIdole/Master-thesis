#!/usr/bin/env python
# image_processing_node.py

import rospy
import cv2
import numpy as np
import imutils
from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import transformations as trans

# Constants
DEPTH_SCALE = 0.001     # Depth is given in integer values on 1mm

class image_processing_node:
	def __init__(self):
		# Object used to convert between ROS images and OpenCV images
		self.bridge = CvBridge()

		self.sub_colour_image = rospy.Subscriber('camera/color/image_raw/compressed', CompressedImage, self.callback_colour_image)
		self.subscriber_camera_info = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.callback_camera_info)
		self.subscriber_odometry = rospy.Subscriber('odom', Odometry, self.callback_odometry)
		rospy.spin()

	def callback_camera_info(self, camera_info):
		self.K = np.array(camera_info.K).reshape([3, 3])

	def callback_odometry(self, odometry):
		self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)

	def callback_depth(self, data):
		print "callback_depth()" # For testing, can be removed
		try:
			# Convert image into OpenCV frame and numpy array
			depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
			depth_array = np.array(depth_image, dtype=np.float32)

			# Find the depth of the centre pixel in metres
			center_idx = np.array(depth_array.shape) / 2
			print "center depth: {0}".format( DEPTH_SCALE * depth_array[center_idx[0], center_idx[1]])

			# Display the result
			cv2.imshow('Depth Image', depth_image)
			cv2.waitKey(2)
		except CvBridgeError, e:
			print e

# Imported from beacon_node.py in starter files
	def callback_colour_image(self, colour_image):
		#print"color"
		rospy.loginfo("Received colour image")
		# Convert to NumPy and OpenCV formats
		colour_np_arr = np.fromstring(colour_image.data, np.uint8)
		colour_cv_frame = cv2.imdecode(colour_np_arr, cv2.IMREAD_COLOR)

		blurred = cv2.GaussianBlur(colour_cv_frame, (11, 11), 0)

		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		colour_lower = (82, 129, 46)
		colour_upper = (100, 255, 255)

		mask = cv2.inRange(hsv, colour_lower, colour_upper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = imutils.grab_contours(contours)
		if len(contours) == 0:
			return
		largest_contour = max(contours, key=cv2.contourArea)
		x, y, w, h = cv2.boundingRect(largest_contour)
		colour_cv_frame = cv2.rectangle(colour_cv_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

		# Display the image
		cv2.imshow('Colour Image', colour_cv_frame)
		#cv2.imshow('Masked Image', mask)
		cv2.waitKey(2)

if __name__ == '__main__':
	try:
		rospy.init_node('image_processing_node')
		ipn = image_processing_node()
	except rospy.ROSInterruptException:
		pass