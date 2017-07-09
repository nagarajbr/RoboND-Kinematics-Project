---
[//]: # (Image References)

[image1]: ./misc_images/misc4.png
[image2]: ./misc_images/eq1.png
[image3]: ./misc_images/eq1.png
[image4]: ./misc_images/image1.png
[image5]: ./misc_images/rot.png

### Writeup / README


#### 1. This is the writeup, you are reading this.
### Kinematic Analysis

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We will begin with initial reference frame of the kuka arm in ints initial position. The schematic diagram represents this initial spawn position, and pose of the arm.

![alt_text][image4]

The DH parameter table has been built using the URDF file. Essentially, the DH table has the following compnents and modifiede paramters:

*alpha:* The twist angle betweeen the zth i-1 and i axes

*a:* The distance between adjacent z axes (i-1 and i) measured along the x axis

*d:* The distance between adjacent x axes (i-1 and i) measured along the z axis

*q:* The joint angles
Each row in the modified DHS parameter table below represents the transformation from frame i to frame i+1.

Link(i) | alpha(i-1) | a(i-1) | d(i) | q(i) 
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1               
2 | -pi/2 | 0.35 | 0 | q2 - pi/2
3 | 0 | 1.25 | 0 | q3
4 | -p1/2 | -0.054 | 1.50 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
Gripper | 0 | 0 | 0.303 | 0

The adjacemt z axes (i-1 and i)can be seen to either be colinear, perpendicular or parallel. This helps derive the values for the DH table above, and will also help perform kinematic analysis in the next steps.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.####

Each link is composed of 4 individual trasnforms, 2 rotations, 2 translations and performed in a order represented in the equation below. We will use this to go step by step, starting from the base link to the end effector, with each step representing a row in the DH table that we derived earlier.

The individual transformation approach:

![alt_text][eq2]

In matrix form the above equation can be represented as:

![alt_text][eq1]


