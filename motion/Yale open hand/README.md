# Introduction and part modification

## Introduction

I use Yale OpenHand Project as my Baxter customized replacement for the default gripper. For Model T, it can do a more advanced grab action due its adaptive finger and a floating pully tree transmission mecanism. More information can be found in its web site [Yale OpenHand Project](https://www.eng.yale.edu/grablab/openhand/).

## Modification

Although the hand project did a good job and some teams have already add it to baxter successfully, I still have to do some modifications to make it work properly.

### Design Options

There are three design options which are Flexure-Flexure(FF), Pivot-Flexure(PF) and Pivot-Pivot(PP). All of them use SDM to generate a rubber layer to increase surface friction and for FF and PF it is also used to connect the finger parts.  
![alt text](https://github.com/zhouyuan7/Baxter-project-3/blob/master/motion/gif/Finger.png)

