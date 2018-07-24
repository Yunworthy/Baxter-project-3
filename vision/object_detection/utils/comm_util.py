#!/usr/bin/env python2

from rospy_message_converter import json_message_converter
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
import json
import ast
from random import randint

cleanup = ":%1234567890"
def bbox_for_publish(obj_Locs):
	keys=sorted(obj_Locs.keys())
	obj_LocsL=[]
	for k in keys:
		#bbox in form of (xmin,ymin,xlength,ylength)
		obj_LocsL.append([(''.join([i for i in k if not i in cleanup])).strip(),(obj_Locs[k][1],obj_Locs[k][0],obj_Locs[k][3]-obj_Locs[k][1],obj_Locs[k][2]-obj_Locs[k][0])])

	#json_str=json.dumps({"data":obj_Locs})
	#message=json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
	message=json.dumps(obj_LocsL)
	return(message)

def bbox_for_sub(message):
	json_str=json_message_converter.convert_ros_message_to_json(message)
	obj_Locs=ast.literal_eval(json.loads(json_str).values()[0])
	for o in obj_Locs:
		o[1]=tuple(o[1])
		o[0]=o[0].encode("ascii")
	return(obj_Locs)

def bbox_adjust(obj_Locs):
	for o in obj_Locs:
		for j in obj_Locs:
			if o!=j and o[0]==j[0] and abs(o[1][0]-j[1][0]) < o[1][2]/2 and abs(o[1][1] - j[1][1]) < o[1][3]/2:
				x=min(o[1][0],j[1][0])
				y=min(o[1][1],j[1][1])
				x_d=max(o[1][0]+o[1][2],j[1][0]+j[1][2])
				y_d=max(o[1][1]+o[1][3],j[1][1]+j[1][3])
				obj_Locs.remove(o)
				obj_Locs.remove(j)
				obj_Locs.append([o[0],(x,y,x_d-x,y_d-y)])
	return(obj_Locs)

def bbox_for_match(data):
	for d in data:
		d[1]=(d[1][0],d[1][1],d[1][0]+d[1][2],d[1][1]+d[1][3])
	message=json.dumps(data)
	return(message)

def assignID(data):
	elementName=data[0][:1]
	elementNum= "%04d" % randint(0,9999)
	ID='#'+elementName+str(elementNum)
	return(ID)
