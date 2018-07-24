#!/usr/bin/env python

import rospy
import roslib
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import tensorflow as tf
from models import getModel
import time
import datetime
import pickle

from utils import visualization_utils
from utils import preProcess_util
from utils import hypercol_util
from utils import keyPoints_util

from test import hcol_test

class image_classification():

	def __init__(self):
		self.count=0 #for testing
		#initialize ROS Publisher (data and output detection feed) and Subscriber (input camera feed)
		self.object_pub = rospy.Publisher("/objects_detected", String, queue_size=1)
		#self.image_pub = rospy.Publisher("/image_detection", Image, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
		self.image_available = False
		#initialize Tensorflow, check to ensure it works
		try:
			self.detection_graph = getModel.loadModel()
			self.category_index = getModel.loadLMap()
			self.detection_graph.as_default()
			self.sess = tf.Session(graph=self.detection_graph)
			self.sessRun = True
		except:
			self.sessRun = False
			print("Tensorflow Session Failed")

	def image_callback (self, data):
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			self.image_available=True
		except CvBridgeError:
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

			net=preProcess_util.network(self.detection_graph,self.sess)
			outputs = ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'c13']

			# Actual detection.
			(boxes, scores, classes, num_detections,outputs) = self.sess.run(
			 [boxes, scores, classes, num_detections,[net[i] for i in outputs]],
			 feed_dict={image_tensor: image_np_expanded})

			# Visualization of the results of a detection.
			visualization_utils.visualize_boxes_and_labels_on_image_array(
			 image_np,
			 np.squeeze(boxes),
			 np.squeeze(classes).astype(np.int32),
			 np.squeeze(scores),
			 self.category_index,
			 use_normalized_coordinates=True,
			 line_thickness=8,
			 draw=False)
#---------------------------------------------------------------------------------
		self.object_pub.publish(' | '.join(visualization_utils.objects))
		#try:
		# 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_np, 'bgr8'))
		#except CvBridgeError:
		#   	print(CvBridgeError)
#---------------------------------------------------------------------------------
		for o in visualization_utils.objects:
			obj_image = image_np[visualization_utils.objLocs[o][0]:visualization_utils.objLocs[o][2], visualization_utils.objLocs[o][1]:visualization_utils.objLocs[o][3]] #Crops the image to focus on detected object
			kps = keyPoints_util.getKeypoints(obj_image,100) #finds keypoints on object
			kps_fixed = [] #adjusts keypoints to be in correct places on raw image
			for k in kps:
				position=(int(k.pt[0])+visualization_utils.objLocs[o][1], int(k.pt[1])+visualization_utils.objLocs[o][0])
				kps_fixed.append(position)
			#hcol_test.printHC(image_np,o,"bottle",kps_fixed,outputs,self.count)
			if "bottle" in o:
				pickle.dump(image_np,open('image.bin','wb'))
				pickle.dump(outputs,open('outputs.bin','wb'))
				pickle.dump(kps_fixed,open('keypoints.bin','wb'))
				#crop = cv2.circle(obj_image,(int(k.pt[0]),int(k.pt[1])),5,color=(255,0,0))
				#cv2.imwrite('keypoint.png',crop)
				#full = cv2.circle(image_np,position,5,color=(255,0,0))
				#cv2.imwrite('full.png',full)
		self.count = self.count+1
#---------------------------------------------------------------------------------

def main(args):
	rospy.init_node('object_detection', anonymous=True)
	print("--Node initiated--")	
	imgcl=image_classification()
	while not rospy.is_shutdown():
		if imgcl.image_available == True:
			imgcl.process_current_image()	
	print("--Shutting down node--")
	cv2.destroyAllWindows()
		
if __name__ == "__main__":
	main(sys.argv)
