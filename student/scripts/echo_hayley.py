#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Echo:
	def __init__(self):
		self.node_name="Echo"
		self.sub_img=rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbEcho)
		self.pub_img=rospy.Publisher("echo_img", Image, queue_size=1)
		self.bridge=CvBridge()


if __name__=="__main__": 
	rospy.init_node('Echo')
	yay = Echo()
	rospy.spin()
