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
import sys
import os
import rospy
import time
import numpy as np
import json
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils import comm_util
from utils import visualization_utils

cwd = os.getcwd()+'/src/object_detection'
os.chdir(cwd)

class image_tracking():
	def __init__(self,feed):
		#Takes the image passed down from the object_detection node
		self.image_sub=rospy.Subscriber(feed+"/image_detection", Image, self.image_callback,queue_size=1, buff_size=2**24)
		#Takes the information about the object locations fromt he object detection node
		self.bbox_sub=rospy.Subscriber(feed+"/bbox",String,self.bbox_callback,queue_size=1, buff_size=2**24)

		#Publishes the image with the tracking data
		self.image_pub=rospy.Publisher(feed+"/image_track",Image,queue_size=1)
		#Used to convert the subscribed image into a usable format for OpenCV
		self.bridge=CvBridge()
		#Publishes the updated object locations along with tracking information
		self.bboxU_pub=rospy.Publisher(feed+"/bbox_update",String,queue_size=1)
		#Prevents script from running unless data is available
		self.image_available=False
		self.bbox_available=False
		#various variables to help the script run (explained below)
		self.init=False	#used to run an initial set of functions that can't be put up here	
		self.timer=time.time()
		self.oTrack=[]

	def image_callback(self,data):
		#Takes the image from the subscriber and converts it into an OpenCV format
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			self.image_available=True
		except CvBridgeError:
			print(CvBridgeError)

	def bbox_callback(self,data):
		#Takes the object information from the ROS publisher and converts it to its origianl form
		self.obj_Locs=comm_util.bbox_for_sub(data)
		self.obj_Locs=comm_util.bbox_adjust(self.obj_Locs)
		#Adds abother field in order to add tracking numbers
		for o in self.obj_Locs:
			o.append('')


	def process_current_image(self):
		frame=self.image
	#--------------------------------------------------------------------------------------------
		#updates the oTrack variable in order to track accuratley while not affecting the raw data
		if (self.init==False) or (time.time()-self.timer>5):
			#clears all objects from the tracker when a failure occurs
			if self.init==False:
				self.oTrack=[]
			#used to remove objects that are already being tracked
			remove=[]
			#Updates the tracker information for each object while preventing repeats and ensuring continuity with the boxes
			for o in self.obj_Locs:
				for i in range(0,len(self.oTrack)):
					t=self.oTrack[i]
					if (o[0]==t[0]) and abs(o[1][0]-t[1][0]) < o[1][2]/2 and abs(o[1][1] - t[1][1]) < o[1][3]/2:
						self.oTrack[i]=o
						remove.append(o)
						self.oTrack[i][2]=t[2]

			self.oTrack=self.oTrack+self.obj_Locs
			#removes duplicate objects
			for r in remove:
				self.oTrack.remove(r)
			#creates tracker object
			self.mTrack=cv2.MultiTracker.create()
			#adds all items to the tracker to track multiple objects at the same time
			for o in self.oTrack:
				#tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
				ok=self.mTrack.add(cv2.TrackerKCF_create(),frame,o[1])

			self.init=True
			self.timer=time.time()
		#generates the tracked object as they move in the frame
		ok,boxes=self.mTrack.update(frame)
		count=0
		#draws the boxes, assign tracking ID's, and updates the tracking information with the new locations
		for newBox in boxes:
			self.oTrack[count][1]=tuple(newBox.tolist())
			if len(self.oTrack[count][2]) == 0:
				self.oTrack[count][2]=comm_util.assignID(self.oTrack[count])
			if ok:
				# Tracking success
				visualization_utils.draw_bounding_box_on_image_array(
				 frame,
				 int(newBox[1]),
				 int(newBox[0]),
				 int(newBox[1] + newBox[3]),
				 int(newBox[0] + newBox[2]), 
				 color='red', 
				 display_str_list=[self.oTrack[count][2]],
				 use_normalized_coordinates=False,
				 draw=True)
			else:
				# Tracking failure
				self.init=False
			count=count+1
	#--------------------------------------------------------------------------------------------
		#pulishes the new object information with updated locations and tracking numbers
		self.bboxU_pub.publish(json.dumps(self.oTrack))	
		#publishes the image for use in other nodes
		try:
		 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
		except CvBridgeError:
		   	print(CvBridgeError)

def main(args):
	#Starts the node
	feed = raw_input("Enter name of cv_camera_node (No quotes, enter if unnamed): /")
	if len(feed)>0:
		feed="/"+feed
	print("--Node: Object Tracking Started--")
	rospy.init_node('object_matching', anonymous=True)
	imgtk=image_tracking(feed)
	while not rospy.is_shutdown():
		if imgtk.image_available == True:
			imgtk.process_current_image()	
	#shutsdown the node
	print("--Shutting Down--")
	cv2.destroyAllWindows()
		
if __name__ == "__main__":
	main(sys.argv)
