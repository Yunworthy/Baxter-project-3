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
from sensor_msgs.msg import CompressedImage
import numpy as np
import tensorflow as tf
from models import getModel

from utils import visualization_utils
from utils import comm_util

cwd = os.getcwd()+'/src/object_detection' #ensures directory starts in the ROS package folder
os.chdir(cwd)

class image_classification():

	def __init__(self,feed): #runs once on start-up
		#initialize ROS Publishers and Subscribers:
		#shows objects detected in frame as text
		self.object_pub = rospy.Publisher(feed+"/objects_detected", String, queue_size=1) 
		#shows frame after detections/matches are performed/drawn
		self.image_pub = rospy.Publisher(feed+"/image_detection", Image, queue_size=1)
		#used to publish bounding boxes/object for other use
		self.bbox_pub = rospy.Publisher(feed+"/bbox",String,queue_size=1)
		#needed to convert ros Image message into usable format for OpenCV
		self.bridge = CvBridge() 

		#gathers image data from camera by frame. Queue size and buffer size used to prevent lag in output
		if len(feed)==0:
			feed="/cv_camera"
		self.image_sub = rospy.Subscriber(feed+"/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**24) 

		self.image_available = False #ensures program won't start unless an image is there

		#initialize Tensorflow, check to ensure it works
		try:
			#load all relevant information of the model from the getModel.py script
			self.detection_graph = getModel.loadModel()
			self.category_index = getModel.loadLMap()
			self.detection_graph.as_default()
			#starts a Tensorflow session using the model
			self.sess = tf.Session(graph=self.detection_graph)
			self.sessRun = True
		except:
			#ensures program won't run if the session cannot load
			self.sessRun = False
			print("Tensorflow Session Failed")

	def image_callback (self, data):
		#loads data from the subscribed node (the camera feed) and converts it to usable form
		try:
			np_arr = np.fromstring(data.data, np.uint8)
			self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			self.image_available=True
		except CvBridgeError:
			#ensures program won't run if CVBridge has an issue
			print(CvBridgeError)

	def process_current_image(self):
		if (self.sessRun == True):
	#---------------------------------------------------------------------------------
			# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
			image_np = self.image
			image_np_expanded = np.expand_dims(image_np, axis=0)
			image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			# Each box represents a part of the image where a particular object was detected.
			boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			# Each score represent how level of confidence for each of the objects.
			# Score is shown on the result image, together with the class label.
			scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

			# Actual detection.
			(boxes, scores, classes, num_detections) = self.sess.run(
			 [boxes, scores, classes, num_detections],
			 feed_dict={image_tensor: image_np_expanded})

			# Visualization of the results of a detection.
			visualization_utils.visualize_boxes_and_labels_on_image_array(
			 image_np,
			 np.squeeze(boxes),
			 np.squeeze(classes).astype(np.int32),
			 np.squeeze(scores),
			 self.category_index,
			 min_score_thresh=.7,
			 use_normalized_coordinates=True,
			 line_thickness=8,
			 draw=False)
			#Packages the object location information to be sent over a ROS publisher
			self.bbox_pub.publish(comm_util.bbox_for_publish(visualization_utils.objLocs))
	#---------------------------------------------------------------------------------
		#Sends messages of detected objects to ROS
		self.object_pub.publish(' | '.join(visualization_utils.objects))

		#Sends images of process frames to ROS, ensures that CVBridge is still working
		try:
		 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_np, 'bgr8'))
		except CvBridgeError:
		   	print(CvBridgeError)

def main(args):
	#Names and creates node
	feed = raw_input("Enter name of cv_camera_node (No quotes, enter if unnamed): /")
	if len(feed)>0:
		feed="/"+feed
	rospy.init_node('object_detection', anonymous=True)
	print("--Node: Object Detection Started--")
	imgcl=image_classification(feed)
	while not rospy.is_shutdown(): #will run in a continuous loop until killed
		if imgcl.image_available == True: #to prevent errors from missing image
			imgcl.process_current_image()	
	print("--Shutting Down--")
	cv2.destroyAllWindows() #closes out the node
		
if __name__ == "__main__":
	#used to run when script is called
	main(sys.argv)
