#!/usr/bin/env python
import rospy
from sesnor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped #steering messages
from std_msgs.msg import *
from student.msg import blob_detect

class MoveToBlob:	
	angle = 0
	e1 = 0
	#right = True
	death = False


	def __init__(self):

		rospy.init_node("blobListener_node")
		rospy.on_shutdown(self.shutdown)

		rate = 10
		r = rospy.Rate(rate)
		
		#set subscriber
		rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.detect_img)
		rospy.Subscriber("blob_info", blob_detect, blob_callback)

		#set publisher
		self.drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)

		self.img_height = 0
		self.img_width = 0

		speed = 2.0
		dist_trav = 20.0
		
		drive_cmd = AckermannDriveStamped()
		drive_cmd.drive.speed = speed
		drive_cmd.drive.steering_angle = self.angle

		time = dist_trav / speed
		ticks = int(time * rate)
		for t in range(ticks):
			drive_cmd.drive.steering_angle = self.angle

			if self.death:
				drive_cmd.drive.speed = -.1
			elif drive_cmd.drive.speed > 0
				drive_cmd.drive.speed = speed

			self.drive.publish(drive_cmd)
			r.sleep ()
		
	def blob_callback(self, data):

		print("blob found")
		f = data.locations[0]
		print "x:",str(f.x)," y:",str(f.y)," z:",str(f.z)
		#this is where we tell it where to drive

		blob_x = self.img_width * f.x
		blob_y = self.img_height * f.y
		
		curr_x = self.img_width/2
		curr_y = self.img_height/2

		dx = curr_x - blob_x
		dy = curr_y - blob_y

		self.death = (blob_x*blob_y)/(curr_x*curr_y) > .5

		

	#def getSteeringCommand(self, error, fullLeft, fullRight):
		

	def zed_cam_callback(self,data):
		img_data = self.bridge.imgmsg_to_cv2(data)
		self.img_width, self.img_height = cv.Getsize(img_data)



if __name__ == "__main__":
	node = MoveToBlob()
	rospy.spin()
