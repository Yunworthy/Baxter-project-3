#!/usr/bin/env python

#from https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_fast/py_fast.html#fast

import numpy as np
import cv2
from matplotlib import pyplot as plt

def getKeypoints(image, threshold=0):
	if threshold==0:	
		orb = cv2.ORB_create()
	else:
		orb = cv2.ORB_create(threshold)
	kp = orb.detect(image,None)
	return(kp)

def getFASTKeypoints(image, threshold = 0):
	# Initiate FAST object with default values
	if threshold==0:	
		fast = cv2.FastFeatureDetector_create()
	else:
		fast = cv2.FastFeatureDetector_create(threshold)
	# find and draw the keypoints
	kp = fast.detect(image,None)
	#img2 = cv2.drawKeypoints(image, kp, None, color=(255,0,0))

	# Print all default params
	#print "Threshold: ", fast.getInt('threshold')
	#print "nonmaxSuppression: ", fast.getBool('nonmaxSuppression')
	#print "neighborhood: ", fast.getInt('type')
	#print "Total Keypoints with nonmaxSuppression: ", len(kp)
	#cv2.imwrite('fast_true.png',img2)
	return(kp)

	# Disable nonmaxSuppression
	#fast.setBool('nonmaxSuppression',0)
	#kp = fast.detect(img,None)

	#print "Total Keypoints without nonmaxSuppression: ", len(kp)

	#img3 = cv2.drawKeypoints(img, kp, color=(255,0,0))

	#cv2.imwrite('fast_false.png',img3)

