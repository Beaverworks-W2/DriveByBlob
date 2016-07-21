#!/usr/bin/env python
import rospy as rsp
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ZedCamPub:

	def __init__(self):

		self.bridge = CvBridge()
		self.zed_pub = rsp.Publisher("image_echo", Image, queue_size=10)
		
		self.zed_img = rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.detect_img)		
		self.header = std_msgs.msg.Header()

		rsp.init_node("zed_pub")
		
	def detect_img(self, img):
		rsp.loginfo("Image found!")

		img_data = self.bridge.imgmsg_to_cv2(img)

		processed_img_cv2 = self.process_img(img_data)
		processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8")

		self.zed_pub.publish(processed_img)

	def process_img(self, img):
		rsp.loginfo("Image Echoed")
		
		hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
		

		maskGreen = cv2.inRange(hsv, np.array([50,0,0]), np.array([70,0,0])
		maskedGreen = cv2.bitwise_and(hsv, hsv, mask=maskGreen)
		
		ret, thresh = cv2.threshold(maskedGreen, 60, 255, BINARY)
		image, contours, hierarchy = cv2.findContours(THRESH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		imWithContours = cv2.drawContours(image, contours, -1, (120, 0, 0), 4)
		#hsv_fin =  

		imWithContours = cv2.cvtColor(hsv_fin, cv2.COLOR_HSV2RGB)

		return img_fun

if __name__ == "__main__":
	node = ZedCamPub()
	rsp.spin()
