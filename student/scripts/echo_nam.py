#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def callback(data):
	rospy.loginfo("Called back")
	#rospy.loginfo(data


def listener():
	rospy.init_node("echo_nam")
	rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()
