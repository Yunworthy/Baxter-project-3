# Introduction and part modification

<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/hand.gif"/>
</p>
<p align="center">
Performance of Pivot-Pivot finger without rubber layer
</p>
<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-3/blob/master/source/finger_ff.gif"/>
</p>
<p align="center">
Performance of Flexure-Flexure finger
</p>


## Introduction

I use Yale OpenHand Project as my Baxter customized replacement for the default gripper. For Model T, it can do a more advanced grab action due its adaptive finger and a floating pully tree transmission mechanism. More information can be found in its web site [Yale OpenHand Project](https://www.eng.yale.edu/grablab/openhand/).

## Modification

Although the hand project did a good job and some teams have already add it to baxter successfully, I still have to do some modifications to make it work properly.

### Design Options

There are three design options which are Flexure-Flexure(FF), Pivot-Flexure(PF) and Pivot-Pivot(PP). All of them use SDM to generate a rubber layer to increase surface friction and for FF and PF it is also used to connect the finger parts.  
![alt text](https://github.com/zhouyuan7/Baxter-project-3/blob/master/motion/gif/Finger.png)

In this project, I choose both PP and ff structure. For pp structure, in order to simplify the building process, I redisgn the friction layer without rubber layer. I have to admitted that without SDM layer the hand's grabing ability is lower and I highly recommand to add rubber layer.

### CAD&Assembly modification

Yale OpenHand Project provides all CAD .stl files to do 3D printing, but the assembly process suprises me and I have to modify some of the CAD to make the hand work properly.

#### Pivot hole tolerance issue

I'm not sure whether this is a printer issue or a design issue, but the result is the default finger assembly tolerance is too tight and the finger can't close properly. I modeify the CAD to enlarge the hole and make it work proerly.

#### Fish line hole issue

The default design use drill press to make tiny fish line holes. I found out that the 3D printer in our lab can make these holes during the printing process, so I modify the CAD to add the hole inside them.

#### Print pose issue

Some of the parts' important surfaces may 'floating' in default printing pose and will make the quality of these surfaces very bad so be careful when you choose the printing pose. 

#### Tie the knot issue

The transmission in all three hand modules are using fish line. In this kind of system, the stability of the knot is very crucial. The lossing of the knot is unacceptable. I try my best to fix this issue but the stretching of the line is still a open question. The stretching phenomenon weaken the range of hand finger movement, so impair the total performance.

### Motor controll

All three models use DYNAMIXEL MX-64T servo actuator [MX-64T](https://www.trossenrobotics.com/p/mx-64t-dynamixel-robot-actuator.aspx). To control it, I recommend to use [U2D2](http://www.robotis.us/u2d2/) which is a small size USB communication converter that enables to control and operator DYNAMIXEL with PC. 
U2D2 does not supply the power to the motor, so a separate power supply is needed and I recommend SMPS2 Dynamixel Adapter [SMPS2 Dynamixel Adapter](https://www.trossenrobotics.com/store/p/5886-SMPS2Dynamixel-Adapter.aspx).

The motor can be controlled in ROS, and there is a package called [dynamixel_controllers](http://wiki.ros.org/dynamixel_controllers)


All in all, Yale OpenHand Peoject did a great job and I was very impressed by its floating pully tree transmission mechanism. However, this is my first time using 3D printing so it was not an easy job, but I do learn a lot from 3D printing skill and CAD drawing.
