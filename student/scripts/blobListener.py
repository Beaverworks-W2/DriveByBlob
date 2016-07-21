#!/usr/bin/env python
import rospy
from sesnor_msgs.msg import Image
from std_msgs.msg import *
from student.msg import blob_detect

class MoveToBlob:	

	def __init__(self):

		rospy.init_node("blobListener_node")
		
		rospy.Subscriber("blob_info", blob_detect, blob_callback)

		rospy.spin()

	def blob_callback(self, data):

		print("blob found")
		f = data.locations[0]
		print "x:",str(f.x)," y:",str(f.y)," z:",str(f.z)

	def zed_cam_callback(self,data):



if __name__ == "__main__":
	node = MoveToBlob()
	rospy.spin()
