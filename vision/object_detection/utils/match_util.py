#!/usr/bin/env python2

#from https://docs.opencv.org/3.3.0/da/df5/tutorial_py_sift_intro.html

import cv2
import numpy as np
import os
from utils import visualization_utils
from memory import loadMemory
import time
cwd=os.getcwd()
memdir=cwd+'/src/object_detection/memory/'
cleanup = ":%1234567890"

def kpDetect(image,data,serialize=False):
	object_data = [] #2D array of [object,kp,desc] (Maybe convert to dictionary?)
	for o in data:
		mask = np.zeros(image.shape,np.uint8) #creates zero image of image_np
		mask[int(o[1][1]):int(o[1][3]), int(o[1][0]):int(o[1][2])] = image[int(o[1][1]):int(o[1][3]), int(o[1][0]):int(o[1][2])] #places ROI in zero image, creating masked image
		gray=cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY) #computing keypoints and descriptors
		sift = cv2.xfeatures2d.SIFT_create(50)
		kp,desc = sift.detectAndCompute(gray,None)
		if serialize==True: #for serialization if needed		
			kp = kpConvert(kp)
			desc=descConvert(desc)
		object_data.append([o[0],kp,desc])
	return(object_data)

def kpConvert(kp,serial=True):
	if serial == True:
		index=[]
		for point in kp:
			temp = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
			index.append(temp)
		return(index)
	if serial == False:
		kp=[]
		for point in index:
			temp = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5]) 
			kp.append(temp)

def descConvert(desc,array=True):
	if array==True:
		return(desc.tolist())
	if array==False:
		return(np.asarray(desc))

def kpDraw(image,object_data):
	for o in object_data:
		image=cv2.drawKeypoints(image,o[1],None)
	return(image)

def kpBFMatchTest(image_np, object_data):
	if len(object_data)>0:
		base=cv2.imread("test/tMug.jpg")
		base=np.uint8(base)
		gray=cv2.cvtColor(base,cv2.COLOR_BGR2GRAY)
		sift = cv2.xfeatures2d.SIFT_create()
		kp,desc = sift.detectAndCompute(gray,None)

		image=image_np
		kpTotal=[]
		descTotal=object_data[0][2]
		f = False
		for o in object_data:
			kpTotal = kpTotal+o[1]
			if f == True:
				descTotal = np.concatenate((descTotal,o[2]))
			f=True

		bf = cv2.BFMatcher()
		matches = bf.knnMatch(descTotal,desc, k=2)
		good = []
		for m,n in matches:
		    if m.distance < 0.6*n.distance:
			good.append([m])
		#queryIdx refers to descTotal and trainIdx refers to desc
		'''if len(good)>15:
			print("I've seen this cup before")
		else:
			print("I haven't seen this cup before")'''
		return(cv2.drawMatchesKnn(image,kpTotal,base,kp,good,None,flags=2))
	else:
		return(image_np)

def kpFMatchTest(image_np, object_data):
	if len(object_data)>0:
		base=cv2.imread("test/tMug.jpg")
		base=np.uint8(base)
		image=image_np
	
		kpTotal=[]
		descTotal=object_data[0][2]
		f = False
		for o in object_data:
			kpTotal = kpTotal+o[1]
			if f == True:
				descTotal = np.concatenate((descTotal,o[2]))
			f=True
		
		gray=cv2.cvtColor(base,cv2.COLOR_BGR2GRAY)
		sift = cv2.xfeatures2d.SIFT_create()
		kp,desc = sift.detectAndCompute(gray,None)

		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks=50)
		flann = cv2.FlannBasedMatcher(index_params,search_params)
		
		matches = flann.knnMatch(descTotal,desc,k=2)
		matchesMask = [[0,0] for i in xrange(len(matches))]
		for i,(m,n) in enumerate(matches):
			if m.distance < 0.6*n.distance:
				matchesMask[i]=[1,0]

		draw_params = dict(matchColor = (0,255,0),
				   singlePointColor = (255,0,0),
				   matchesMask = matchesMask,
				   flags = 0)
		return(cv2.drawMatchesKnn(image,kpTotal,base,kp,matches,None,**draw_params))
	else:
		return(image_np)

def kpBFMatchTest2(image_np, object_data):
	image=image_np
	if len(object_data)>0:
		for o in object_data:
			if o[0] in loadMemory.listCat():
				objmem=loadMemory.listObj(o[0])
				if len(objmem)>0:
					for item in objmem:
						base=cv2.imread(loadMemory.memdir+'/'+o[0]+'/'+item)
						base=np.uint8(base)
						gray=cv2.cvtColor(base,cv2.COLOR_BGR2GRAY)
						sift = cv2.xfeatures2d.SIFT_create()
						kp,desc = sift.detectAndCompute(gray,None)

						kpFeed=o[1]
						descFeed=o[2]

						bf = cv2.BFMatcher()
						matches = bf.knnMatch(descFeed,desc, k=2)
						good = []
						for m,n in matches:
						    if m.distance < 0.7*n.distance:
							good.append([m])
						if len(good)>15:
							print('I have seen this '+o[0]+' before!')
						else:
							print('I have not seen this '+o[0]+' before...')
				else:
					print('I have not seen this '+o[0]+' before...')
			else:
				print('I have not seen this '+o[0]+' before...')

def smartMatch(object_data,image,location,add_item=True,draw=False):
	outmessage=[] #array of messages pertaining to each object in frame
	duo=False #used to prevent duplicate messages when tracking
	if len(object_data)>0: #ensures that there are objects to use
		for o in object_data:
			if o[0] in loadMemory.listCat(): #looks in memory to see if this type of object has been seen before
				objmem=loadMemory.listObj(o[0]) #loads all of the object files for a specific category
				match=False #check to see if object is in memory
				while match==False:
					for item in objmem: #iterates through each object file
						desc=np.float32((np.loadtxt(memdir+o[0]+'/'+item)))
						descFeed=o[2]
						#desc=from file, descFeed=from camera (loads data from file/from object data)
						bf = cv2.BFMatcher() #matching algorithm
						matches = bf.knnMatch(descFeed,desc, k=2)
						good = []
						other=[]
						for m,n in matches: #looks for all valid matches
						    if m.distance < 0.7*n.distance: #can be calibrated
							good.append([m])
						if len(good)>10: #if there are a certain number of good matches
							if add_item==True: #update the object file with an average of match descriptors
								for g in good:
									desc[g[0].trainIdx]=(descFeed[g[0].queryIdx]+desc[g[0].trainIdx])/2
								np.savetxt(memdir+o[0]+'/'+item,desc,fmt='%d')
							if item[:-4]!=location[object_data.index(o)][3]: #gives the object a matching ID
								outmessage.append('I have seen this '+o[0]+'('+location[object_data.index(o)][2]+') before! ')
								location[object_data.index(o)][3]=item[:-4]
								duo=True
							if draw==True: #draws a successful match icon on the image
								visualization_utils.matchID(True,object_data.index(o),location,image)
							match=True
						if item[:-4]==location[object_data.index(o)][3]:
							#TODO Too many desc get saved
							if add_item==True and len(good)<10: #adds information of tracked object to object file
								desc = np.concatenate((desc,descFeed), axis=0)
								np.savetxt(memdir+o[0]+'/'+item,desc,fmt='%d')
							if duo==False:
								outmessage.append('I am tracking this '+o[0]+'('+location[object_data.index(o)][2]+'). ')
							if draw==True: #draws successful match icon
								visualization_utils.matchID(True,object_data.index(o),location,image)
							match=True
					if match==False: #adds item if never seen before
						outmessage.append("I have not seen this "+o[0]+"("+location[object_data.index(o)][2]+") before. ")
						if draw==True: #draws unsuccessful match icon
							visualization_utils.matchID(False,object_data.index(o),location,image)
						if add_item==True: #creates new object file
							index="%04d" % (int(filter(lambda x: x.isdigit(),item))+1,)
							np.savetxt(memdir+o[0]+'/#'+index+'.txt', o[2], fmt='%d')
							outmessage.append("New item added to "+o[0]+" on: " + str(time.asctime(time.localtime(time.time())))+' ')

					match=True
			else: #adds category if no object match
				if draw==True: #draws unsuccessful match icon
					visualization_utils.matchID(False,object_data.index(o),location,image)
				outmessage.append("I have never seen a(n) "+o[0]+" before. ")
				if add_item == True: #adds new object file to category
					os.makedirs(memdir+o[0])
					np.savetxt(memdir+o[0]+"/#0000.txt",o[2],fmt='%d')
					outmessage.append("New item added to "+o[0]+" on: " + str(time.asctime(time.localtime(time.time())))+' ')
	else:
		outmessage.append("I do not see anything. ")

	return(outmessage)
