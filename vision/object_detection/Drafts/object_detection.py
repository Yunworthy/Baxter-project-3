#!/usr/bin/env python

import cv2
import os
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
from utils import keyPoints_util
from utils import SIFT_util

cwd = os.getcwd()

class image_classification():
	def setup(self):
		self.count=0 #for testing
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

	def getImages(self,imageFname):
		self.image = cv2.imread(imageFname)	

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
			 use_normalized_coordinates=True,
			 line_thickness=8,
			 draw=False)
	#---------------------------------------------------------------------------------
		print(' | '.join(visualization_utils.objects))
		#cv2.imshow('Detection',image_np)
	#---------------------------------------------------------------------------------
		'''for o in visualization_utils.objects:
			mask = np.zeros(image_np.shape,np.uint8) #creates zero image of image_np
			mask[visualization_utils.objLocs[o][0]:visualization_utils.objLocs[o][2], visualization_utils.objLocs[o][1]:visualization_utils.objLocs[o][3]] = image_np[visualization_utils.objLocs[o][0]:visualization_utils.objLocs[o][2], visualization_utils.objLocs[o][1]:visualization_utils.objLocs[o][3]] #places ROI in zero image, creating masked image
			#SIFT_util.kpDetect(image_np,mask,self.count)
			kps = keyPoints_util.getFASTKeypoints(mask,120) #finds keypoints on object
			print("Total keypoints: ",len(kps))
			hcol_test.HCDict(image_np,o,"car",kps,outputs,self.count)
			if 'car' in o:
				self.count = self.count+1'''

def run(images):
	ic = image_classification()
	ic.setup()
	for i in images:
		ic.getImages(i)
		ic.process_current_image()

if __name__=="__main__":
	images=[]
	data_dir = cwd+'/models/CarDataset/'
	imageNames = sorted(os.listdir(data_dir))
	for i in imageNames:
		images.append(data_dir+i) 
	run(images)
