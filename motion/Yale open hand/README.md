# Introduction and part modification

## Introduction

I use Yale OpenHand Project as my Baxter customized replacement for the default gripper. For Model T, it can do a more advanced grab action due its adaptive finger and a floating pully tree transmission mechanism. More information can be found in its web site [Yale OpenHand Project](https://www.eng.yale.edu/grablab/openhand/).

## Modification

Although the hand project did a good job and some teams have already add it to baxter successfully, I still have to do some modifications to make it work properly.

### Design Options

There are three design options which are Flexure-Flexure(FF), Pivot-Flexure(PF) and Pivot-Pivot(PP). All of them use SDM to generate a rubber layer to increase surface friction and for FF and PF it is also used to connect the finger parts.  
![alt text](https://github.com/zhouyuan7/Baxter-project-3/blob/master/motion/gif/Finger.png)

In this project, I choose PP structure without SDM layer in order to simplify the building process. I have to admitted that without SDM layer the hand's grabing ability is lower but it still works well enough.

### CAD&Assembly modification

Yale OpenHand Project provides all CAD .stl files to do 3D printing, but the assembly process do suprise me and I have to modify some of the CAD to make the hand work properly.

#### Pivot hole tolerance issue

I'm not sure whether this is a printer issue or a design issue, but the result is the default finger assembly tolerance is too tight and the finger can't close properly. I modeify the CAD to enlarge the hole and make it work proerly.

#### Fish line hole issue

The default design use drill press to make the tiny fish line hole. I found out that the 3D printer in our lab can make these holes during the printing process, so I modify the CAD to add the hole inside them.

#### Print pose issue

Some of the parts' important surfaces may 'floating' in default printing pose and will make the quality of these surfaces very bad so be careful when you choose the printing pose.

All in all, Yale OpenHand Peoject did a great job and I was very impressed by its floating pully tree transmission mechanism. However, this is my first time using 3D printing so it was not an easy job, but I do learn a lot from 3D printing skill and CAD drawing. 

#### Tie the knot issue

The transmission in all three hand modules are using fish line. In this kind of system, the stability of the knot is very crucial. The lossing of the knot is unacceptable. I try my best to fix this issue but the stretching of the line is still a open question. The stretching phenomenon weaken the range of hand finger movement, so impair the total performance.
