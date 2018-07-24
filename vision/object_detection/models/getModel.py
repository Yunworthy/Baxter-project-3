#!/usr/bin/env python
###
##MUST use Python 2.7 (Python 3 will NOT work)
##Nessecary (all compatible with Python 2.7):
## >Tensorflow (written using version 1.4.0)
##All other imports should be pre-installed with your python package or in the 'object_detection' package
##Written by Brandon Sookraj for Professor Roberto Tron's lab at Boston University
###
import os
import six.moves.urllib as urllib
import sys
import tarfile
import zipfile
import tensorflow as tf
import time
from utils import label_map_util

cwd = os.getcwd()+'/src/object_detection' #makes sure direcotry remains in ROS package folder

def loadModel():
	os.chdir(cwd+'/models')
	#Model preparation 
	#Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_CKPT` to point to a new .pb file.  
	#By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.
	print("--Preparing model--")
	#Name model to download.
	MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017' #must regenerate frozen inference graph for newer models TODO
	#Establishes download file type and location
	MODEL_FILE = MODEL_NAME + '.tar.gz'
	DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

	#Path to frozen detection graph. This is the actual model that is used for the object detection.
	PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

	#Downloads the model if it isn't there. Otherwise it moves on.
	if not os.path.exists(MODEL_NAME + '/frozen_inference_graph.pb'):
		print ('Downloading the model')
		#opens download file from location using model name information
		opener = urllib.request.URLopener()
		opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
		#untars the file
		tar_file = tarfile.open(MODEL_FILE)
		for file in tar_file.getmembers():
		  #only downlaods the frozen inference graph from the tar file
		  file_name = os.path.basename(file.name)
		  if 'frozen_inference_graph.pb' in file_name:
			tar_file.extract(file, os.getcwd())
		print ('Download complete')
	else:
		print ('Model already exists')

	#Load a (frozen) Tensorflow model into memory.
	print("--Loading Model--")
	tic = time.time() #used to tell time taken to load model
	detection_graph = tf.Graph() #creates a graph object
	#reads frozen inference graph from file and loads it to memory
	with detection_graph.as_default():
	  od_graph_def = tf.GraphDef()
	  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
		serialized_graph = fid.read()
		od_graph_def.ParseFromString(serialized_graph)
		tf.import_graph_def(od_graph_def, name='')
	print("--Model loading complete--", time.time()-tic, 'sec')
	os.chdir(cwd)
	#return the loaded graog for use in other scripts
	return(detection_graph)

def loadLMap():
	#Loading label map
	#Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

	#List of the strings that is used to add correct label for each box.
	#Location of the label map and number of classes (90)
	PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
	NUM_CLASSES = 90
	#loads the label map from file, and converts for identification
	label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
	categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
	category_index = label_map_util.create_category_index(categories)
	#returns the list of labels for use in other scripts
	return(category_index)
