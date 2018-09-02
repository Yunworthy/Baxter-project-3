# Baxter-project-3

![alt text](https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/baxter_cover.jpg)

![alt text](https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/baxter_vision0.jpg)

## Introduction

<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/baxter_video_final.gif"/>
</p>

This is my third Baxter project. The propose of this project is using KinectV2 sensor and Yale OpenHand T1 hand to replace the Baxter default arm camera and gripper to do an object detection, localization and grabbing. In detail, use machine learning technique and KinectV2 sensor point cloud to get the mean target translation inside Baxter coordinate system, then control the Baxter limb and customized hand to grab the object.

Through this project I did a full engineering train in Robotic area from hardware calibration, CAD drawing, 3D printing setting, sensor&motor driver setting and coding using python&C++ with ROS. Although the project itself isn’t a research project and techniques I used are all popular and mature, this is a great experience to deeply feel the whole process of a typical robotic project and master some state-of-the-art techniques in robotic area.

This project can be classified into a visual servoing control method. Compared with my previous Baxter projects, this can also be divided into vision and motion subparts, but in both two parts the complexity is much higher than last two projects.

<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/baxter_31.gif"/>
</p>
<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/baxter_30.gif"/>
</p>


## Vison
For vison part, in this project, it can be subdivided into pattern recognition and target location. For pattern recognition, I use machine learning technique to detect the target from RGB frame. In detail, I load a state-of-the-art CNN module to do an object classification and get the target box pixel coordinate. For target localization, I use the power of Kinect sensor which is the point cloud generator and combining with target pixel coordinate to get a mean target origin pose(translation) related to the Kinect origin. After that, let the pose go through the calibration matrix and we finally get a pose inside the Baxter base coordinate system. By the way, in order to fix the sensor, I also design a mount and print it.

## Motion
The core of the motion part is the Yale OpenHand build, assembly and driver setting training. Although there are instructions inside its website, I still learn a lot through this process. During this process, I first taste 3D printing and learn some skills from printing pose to CAD drawing. I modify some of the parts CAD in order to simplify the assembling process. 
The driver setting let me learn a lot about the Dynamixel M-64T step motor knowledge and how to control it through ROS.
