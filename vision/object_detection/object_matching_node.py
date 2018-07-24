#!/usr/bin/env python2
###
##MUST use Python 2.7 (Python 3 will NOT work)
##Nessecary (all compatible with Python 2.7):
## >OpenCV 3
## >Tensorflow (written using version 1.4.0)
## >ROS (written using ROS Kinetic)
##All other imports should be pre-installed with your python package or in the 'object_detection' package
##Written by Brandon Sookraj for Professor Roberto Tron's lab at Boston University
###
import cv2
import os
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

from utils import keyPoints_util
from utils import match_util
from utils import comm_util

cwd = os.getcwd()+'/src/object_detection'
os.chdir(cwd)

class image_matching():
	def __init__(self,feed):
		#used to relay information about matches as messages
		self.matcher_pub = rospy.Publisher(feed+"/matcher",String,queue_size=1) 
		#visualization of matches
		self.image_pub = rospy.Publisher(feed+"/image_matching", Image, queue_size=1)
		#converts image message into usable format for cv2
		self.bridge = CvBridge()

		#takes the passed down image from the tracking node
		self.image_sub = rospy.Subscriber(feed+"/image_track", Image, self.image_callback, queue_size=1, buff_size=2**24)
		#takes the updated object tracking information 
		self.data_sub = rospy.Subscriber(feed+"/bbox_update", String, self.data_callback, queue_size=1, buff_size=2**24)
		#prevents the script from running if there is no data
		self.image_available = False
		self.data_available = False

		self.init=False #used to run an initial set of functions that can't be put up here
		self.tracker='' #used to prevent duplicate messages in matcher publisher
		valid=False
		while valid==False:
			mode = raw_input("Run with a dynamic or static memory (d/s)? ")
			if mode=="d":
				print("Running dynamic memory, will add objects seen to memory and update known objects.")
				self.memMode=True
				valid=True
			elif mode=="s":
				print("Running static memory, will not add new objects to memory or update known objects.")
				self.memMode=False
				valid=True
			else:
				print("Invalid input")
				valid=False

	def image_callback (self, data):
		#Takes the image and converts it to an OpenCV usable format
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			self.image_available=True
		except CvBridgeError:
			print(CvBridgeError)

	def data_callback(self,data):
		#Takes the object tracking information and converts it to its original form
		self.objLocs=comm_util.bbox_for_sub(data)
		#Adjusts the object locations and adds a field for a matching ID
		for o in self.objLocs:
			o[1]=(o[1][0],o[1][1],o[1][0]+o[1][2],o[1][1]+o[1][3])
			o.append('')
		self.data_available=True

	def process_current_image(self):
		image=self.image
		#starts the script off with an initial set of objects to use
		if self.init==False:
			self.objects=self.objLocs
			self.init=True
		else: #updates the list of objects to match the in coming data while retaining the match ID 
			if len(self.objLocs)==0:
				self.objects=self.objLocs
			if len(self.objects) == 0 and (self.objLocs != 0):
				self.objects=self.objLocs

			for i in self.objLocs:
				for j in self.objects:
					if j[2]==i[2]:
						j[:3]=i[:3]
					if not any(j[2] in k for k in self.objLocs):
						self.objects.remove(j)
				if not any(i[2] in k for k in self.objects):
						self.objects.append(i)			
		#extracts the SIFT keypoints and features
		features=match_util.kpDetect(image,self.objects)
		#performs the matching. Change add_item to True for dynamic memory or False for static memory
		object_match = match_util.smartMatch(features,image,self.objects,add_item=self.memMode,draw=True)
		#TODO calibrate better
	
		#publishes the image from the matching process for use elsewhere
		try:
		 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
		except CvBridgeError:
		   	print(CvBridgeError)

		#Sends messages of matching information to ROS
		if len(object_match) >0:
			object_match=sorted(object_match)
			message=''			
			for o in object_match:
				message=message+o
			if object_match != self.tracker:
				self.matcher_pub.publish(message)
				self.tracker=object_match

def main(args):
	#Starts the node
	feed = raw_input("Enter name of cv_camera_node (No quotes, enter if unnamed): /")
	if len(feed)>0:
		feed="/"+feed
	print("--Node: Object Matching Started--")
	rospy.init_node('object_matching', anonymous=True)
	imgmt=image_matching(feed)
	while not rospy.is_shutdown():
		if imgmt.image_available == True and imgmt.data_available==True:
			imgmt.process_current_image()	
	#Shutsdown node
	print("--Shutting Down--")
	cv2.destroyAllWindows()
		
if __name__ == "__main__":
	main(sys.argv)
