#!/usr/bin/env python2

import os

cwd=os.getcwd()
memdir=cwd+'/src/object_detection/memory'

def listCat():
	categories = [dI for dI in os.listdir(memdir) if os.path.isdir(os.path.join(memdir,dI))]
	return(categories)

def listObj(category):
	objects=os.listdir(memdir+'/' + category)
	for o in objects:
		if ".txt" not in o:
			objects.remove(o)
	return(objects)
