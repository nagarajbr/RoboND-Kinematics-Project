---
[//]: # (Image References)

[image1]: ./misc_images/misc4.png
[image2]: ./misc_images/eq1.png
[image3]: ./misc_images/eq2.png
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

![alt_text][image3]

In matrix form the above equation can be represented as:

![alt_text][image2]

We will now write a function using python that can be reused to generate the individual transformation matrix and traverse link by link, and ultimately compute the homogeneoud transformation from the 0th to the gripper. The equations above will be used in the program written below.

We do the prerequisite symbol definition, and translate the DH table shown earlier to be used in our program.


# Define linkoffset symbols for the DH table parameters 
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
# Define linklength symbols for the DH table parameters 
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
# Define twist angles for links as symbols for the DH table parameters 
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
# Define symbols for Joint angles. The q's are basically the theta angles that we will refer to in subsequent discussions

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

dh = {alpha0:     0, a0:      0, d1:  0.75,
    alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
    alpha2:     0, a2:   1.25, d3:     0,
    alpha3: -pi/2, a3: -0.054, d4:  1.50,
    alpha4:  pi/2, a4:      0, d5:     0,
    alpha5: -pi/2, a5:      0, d6:     0,
    alpha6:     0, a6:      0, d7: 0.303,  q7: 0 }
```
```
#Pass arguments corresponding to each row in the DH table

def generateMatrix(alpha, a, q, d):
    m =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return m
```

A code snippet for invoking the function above is shown below. This call is repeated for each individual transform:
```
#Invoke Method as below for each T1_2, T2_3, T3_4, T4_5, T5_6, T6_G

T0_1 = generateMatrix (alpha0,a0, q1, d1).subs(s)

#Output from calling T0_1

Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

```
