#!/usr/bin/env python
import rospy as rs
from sensor_msgs.msg import Image
import cv2
import threading
from cv_bridge import CvBridge

class Echo:
	def __init__(self):
		print "initializing..."
		self.thread_lock = threading.Lock()
		self.bridge = CvBridge()
		rs.Subscriber("/camera/rgb/image_rect_color", Image, self.callback)
		self.img_pub = rs.Publisher("/echo/img", Image, queue_size=5)

	def callback(self,image_msg):
        	thread = threading.Thread(target=self.process_image,args=(image_msg,))
        	thread.setDaemon(True)
        	thread.start()

	def process_image(self, msg):
		if not self.thread_lock.acquire(False):
            		return		

		self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
		self.image = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
		print "mesg"
		self.img_pub.publish(msg)

		self.thread_lock.release()
rs.init_node("echo")
node = Echo()
rs.spin()
