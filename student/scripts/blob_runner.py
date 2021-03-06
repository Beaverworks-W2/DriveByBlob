#!/usr/bin/env python
import rospy
from sesnor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped #steering messages
from std_msgs.msg import *
from student.msg import blob_detect
import sys


class MoveToBlob:

	def __init__(self):
	        self.angle = 0
	        self.death = False
	        rospy.init_node("blobListener_node")        
		rate = 10
	        r = rospy.Rate(rate)
        
	        #set subscriber
	        rsp.Subscriber("/camera/rgb/image_rect_color", Image, self.zed_cam_callback)
	        rospy.Subscriber("blob_info", blob_detect, self.blob_callback)        
		#set publisher
	        self.drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)        
		self.stop = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed=0.0)


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

		error = dx        
		self.angle = self.getPid(error)
        	new_steering_angle = self.angle        
		
		self.death = (blob_x*blob_y)/(curr_x*curr_y) > .5        
		
		self.img_height = 0
        	self.img_width = 0
        
		speed = 2.0
	        dist_trav = 20.0  
      
		driver = AckermannDrive()   
             
		driver.speed = speed
        	driver.steering_angle = self.angle

        	drive_cmd = AckermannDriveStamped(self.header, driver)  
      
		time = dist_trav / speed
        	ticks = int(time * rate)

        	for t in range(ticks):

            		drive_cmd.drive.steering_angle = self.angle            
			if self.death:
		                #drive_cmd.drive.speed = -.1
		                self.drive.publish(stop)
	        	elif drive_cmd.drive.speed > 0
		                drive_cmd.drive.speed = speed
                		self.drive.publish(drive_cmd)
            
            		r.sleep ()

	def getPid(self, error):
        	kp = float(sys.argv[1])
        	return kp*float(error)

    	def zed_cam_callback(self,data):
        	img_data = self.bridge.imgmsg_to_cv2(data)
        	self.img_width, self.img_height = cv.Getsize(img_data)

if __name__ == "__main__":
    node = MoveToBlob()
    rospy.spin()





