# Object Detection

This is part of the tools for vision detection, I mainly reference Brandon's object detection from Boston University Robotic lab.

The detection process is done inside the RGB frame. It gives the target box pixel coordinate. Using this coordinate, we map it to the relative point cloud frame to get the target mean coordinate.  

## RGB resolution issue

The Kinect v2 driver provides three type of resolution which are sd, qhd and hd. Due to the limitation of my computer graphic card, I first choose the qhd resolution, but it looks like that the detector doesn’t fit with that high resolution and the performance is very poor, so I finally choose the sd.  
However, I am not sure whether is issue of the driver or myself, the sd RGB image topic inside the ros doesn’t provide the color image but the point cloud image. In order to get the sd RGB frame. I write a script that subscribe the qhd color frame and use cv_bridge to cut the frame and publish it out. Notice that this detector needs the ‘compressImage’ message type, so be careful when you send the stream to it.  

