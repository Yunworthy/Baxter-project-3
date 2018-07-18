#!/usr/bin/env python
import sys
import tf
import math
import rospy
import time
import struct
import numpy as np
import baxter_interface
import baxter_pykdl
import baxter_external_devices
import argparse

 
from std_msgs.msg import Header
from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import (Point,PoseStamped,Pose,Quaternion)

from pynput.keyboard import Key, Controller

def TestVisionTest(XPosition, YPosition, ZPosition):
        right_arm = baxter_interface.Limb('right')
        ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=XPosition,
                        y=YPosition,
                        z=ZPosition,
                    ),
                    orientation=Quaternion(
                        x=-0.377481491084,
                        y=0.924451532113,
                        z=0.0193834993405,
                        w=0.0193834993405,
                    ),
                ),
            ),
        }
 
        ikreq.pose_stamp.append(poses['right'])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
 
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
 
        joint_angles = {'right_s0': limb_joints['right_s0'], 'right_s1': limb_joints['right_s1'], 'right_w0': limb_joints['right_w0'], 'right_w1': limb_joints['right_w1'], 'right_w2': limb_joints['right_w2'], 'right_e0': limb_joints['right_e0'], 'right_e1': limb_joints['right_e1']}
 
        print('Start moving')
        right_arm.move_to_joint_positions(joint_angles)
        print('finished')

def InputToDirection(XPosition, YPosition, ZPosition):
    
    def SetY(YPosition, delta):
        YPosition = YPosition + delta
        TestVision(XPosition, YPosition, ZPosition)

    def SetX(XPosition, delta):
        XPosition = XPosition + delta
        TestVision(XPosition, YPosition, ZPosition)

    def SetZ(ZPosition, delta):
        ZPosition = ZPosition + delta
        TestVision(XPosition, YPosition, ZPosition)

    def TestVision(XPosition, YPosition, ZPosition):
        right_arm = baxter_interface.Limb('right')
        ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=XPosition,
                        y=YPosition,
                        z=ZPosition,
                    ),
                    orientation=Quaternion(
                        x=0.972281927322,
                        y=0.0624294560238,
                        z=0.0104489801262,
                        w=-0.0386312553112,
                    ),
                ),
            ),
        }
 
        ikreq.pose_stamp.append(poses['right'])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
 
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
 
        joint_angles = {'right_s0': limb_joints['right_s0'], 'right_s1': limb_joints['right_s1'], 'right_w0': limb_joints['right_w0'], 'right_w1': limb_joints['right_w1'], 'right_w2': limb_joints['right_w2'], 'right_e0': limb_joints['right_e0'], 'right_e1': limb_joints['right_e1']}
 
        print('Start moving')
        right_arm.move_to_joint_positions(joint_angles)
        print('finished_record_to_txt')

    bindings = {
    #   key: (function, args, description)

        'w': (SetY, [YPosition, +0.05], "Y direction increase"),
        's': (SetY, [YPosition, -0.05], "Y direction decrease"),
        'a': (SetX, [XPosition, +0.05], "X direction increase"),
        'd': (SetX, [XPosition, -0.05], "X direction decrease"),
        'q': (SetZ, [ZPosition, +0.05], "Z direction increase"),
        'e': (SetZ, [ZPosition, -0.05], "Z direction decrease"),

     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        XPosition = XPosition
        YPosition = YPosition
        ZPosition = ZPosition
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
 
def main(args):
    rospy.init_node('Motion', anonymous=True)

    #TestVisionTest(0.724036966537,-0.0552060635377,0.144624583263)

    InputToDirection(0.65,-0.25,0.3)

        

 
if __name__ == '__main__':
    main(sys.argv)
