#!/usr/bin/env python2

import cv2
import sys
import os
import rospy
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils import comm_util

cwd = os.getcwd()+'/src/object_detection'
os.chdir(cwd)

class image_tracking():
	def __init__(self):
		self.image_sub=rospy.Subscriber("/image_detection", Image, self.image_callback,queue_size=1, buff_size=2**24)
		self.bbox_sub=rospy.Subscriber("/bbox",String,self.bbox_callback,queue_size=1, buff_size=2**24)
		self.image_pub=rospy.Publisher("/image_track",Image,queue_size=1)
		self.bridge=CvBridge()
		self.image_available=False
		self.bbox_available=False
		self.init=False
		self.timer=time.time()
		self.oTracker=[]

	def image_callback(self,data):
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			self.image_available=True
		except CvBridgeError:
			print(CvBridgeError)

	def bbox_callback(self,data):
		self.obj_Locs=comm_util.bbox_for_sub(data)

	def process_current_image(self):
		track=True
		frame=self.image
	#--------------------------------------------------------------------------------------------
		if (self.init==False):
			self.oTracker=self.obj_Locs
			self.mTrack=cv2.MultiTracker.create()
			for o in self.obj_Locs:
				#tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
				ok=self.mTrack.add(cv2.TrackerMIL_create(),frame,o[1])
			self.init=True
			self.timer=time.time()
	#--------------------------------------------------------------------------------------------
		if time.time()-self.timer>.5:
			removal=self.oTracker
			for o in self.obj_Locs:
				for i in range (0,len(self.oTracker)):
					t=self.oTracker[i]
					if ((abs(np.subtract(o[1],t[1])[0]) < o[1][2]) and (abs(np.subtract(o[1],t[1])[1]) < o[1][3])):
							removal.remove(t)
			
			if len(self.oTracker)!=0:
				for r in removal:
					self.obj_Locs.append(r)
			self.oTracker=self.obj_Locs
			self.mTrack=cv2.MultiTracker.create()
			for o in self.obj_Locs:
				#tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
				ok=self.mTrack.add(cv2.TrackerKCF_create(),frame,o[1])
			self.timer=time.time()
	#--------------------------------------------------------------------------------------------
		ok,boxes=self.mTrack.update(frame)
		if len(boxes)>0:
			boxes=boxes.tolist()
		tabs=self.oTracker
		for newBox in boxes:
			index=boxes.index(newBox)
			if ok:
				self.oTracker[index][1]=tuple(newBox)
				# Tracking success
				p1 = (int(newBox[0]), int(newBox[1]))
				p2 = (int(newBox[0] + newBox[2]), int(newBox[1] + newBox[3]))
				cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
			else :
				# Tracking failure
				tabs=[subl for subl in tabs if subl[1]==newBox]
				track=False

		self.oTracker=tabs
		if track==False:
			self.mTrack=cv2.MultiTracker.create()
			for o in self.oTracker:
				#tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
				ok=self.mTrack.add(cv2.TrackerKCF_create(),frame,o[1])
				
		try:
		 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
		except CvBridgeError:
		   	print(CvBridgeError)

def main(args):
	rospy.init_node('object_matching', anonymous=True)
	imgtk=image_tracking()
	while not rospy.is_shutdown():
		if imgtk.image_available == True:
			imgtk.process_current_image()	
	cv2.destroyAllWindows()
		
if __name__ == "__main__":
	main(sys.argv)
