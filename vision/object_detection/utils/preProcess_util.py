#!/usr/bin/env python
import tensorflow as tf
import numpy as np
from models.info import getTlength as gtl

def network(graph,sess):
	targets = gtl.target("pointwise/Relu6:0")
	net = {}
	net['x'] = gtl.target("image_tensor:0")
	for i in range(0,len(targets)):
		net['c'+str(i+1)] = graph.get_tensor_by_name(targets[i][0])
	return(net) 

def pixMap(image):
	pix = np.ndarray(shape=(2,),buffer=np.array(image),dtype=float)
	return(pix.T)
