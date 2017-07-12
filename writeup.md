---
[//]: # (Image References)

[image1]: ./misc_images/misc4.png
[image2]: ./misc_images/eq1.png
[image3]: ./misc_images/eq2.png
[image4]: ./misc_images/image1.png
[image5]: ./misc_images/rot.png
[image6]: ./misc_images/codecogseqn.gif
[image7]: ./misc_images/misc5.png
[image8]: ./misc_images/misc6.png

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

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Each link is composed of 4 individual trasnforms, 2 rotations, 2 translations and performed in a order represented in the equation below. We will use this to go step by step, starting from the base link to the end effector, with each step representing a row in the DH table that we derived earlier.

The individual transformation approach:

![alt_text][image3]

In matrix form the above equation can be represented as:

![alt_text][image2]

We will now write a function using python that can be reused to generate the individual transformation matrix and traverse link by link, and ultimately compute the homogeneoud transformation from the 0th to the gripper. The equations above will be used in the program written below.

We do the prerequisite symbol definition, and translate the DH table shown earlier to be used in our program.

``` python
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
The function snippet is below:
``` python
#Pass arguments corresponding to each row in the DH table

def generateMatrix(alpha, a, q, d):
    m =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return m
```

A code snippet for invoking the function above is shown below. This call is repeated for each individual transform:

``` python
#Invoke Method as below for each T1_2, T2_3, T3_4, T4_5, T5_6, T6_G

T0_1 = generateMatrix (alpha0,a0, q1, d1).subs(dh)

#Output from calling T0_1

Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

```
We finally compute the homogeneous transformation from the base link to the gripper by:
``` python
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
We now move on to account for the difference in the orientation of the gripper link frame to the T0_G that we arrived above. We arrive at this by performing a 2 step intrinsic rotations on the gripper frame (z by 180 degrees and y by -90 degrees), and thereby aligning with base link frame. The schematic representation is shown below:

![alt_text][image5]

The code snippet for the correction process described above is shown below:
``` python
# Rotation adjustment about Z

def rotateZ(q):
    Rz = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return Rz
# Rotation adjustment about Y
def rotateY(q):
    Ry = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return Ry

# Calculate the rotational correction from z and y at the gripper link frame

rCorrection = simplify (Rz*Ry)

# Final FW total homgeneou transformation and computation

totalTransform = simplify (T0_G * rCorrection)

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles

We  begin with solving the Inverse position problem. Since the joints 4,5, 6 of the kuka arm are revolute, and the joint axes intersect at joint5, it becomes the common intersection point, and thereby the wrist center. This is a 5 step process, with Step 1 including the symbol definition and DH table creation. We will begin with Step 2 in this section.

##### Step 2: to find the location of the WC relative to the base frame
``` python
#.303 is the d value for the gripper row, which is the length between the gripper and the wrist center (It is 0 between the #wrist center and 6)
DG = dh[d7]  

#Wrist Center Position w.r.t base link
R0_G = tf.transformations.quaternion_matrix([req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
O5_0 = PG_0 - Matrix(DG*R0_G)

```
##### Step 3: find joint variables, q1, q2 and q3, such that the WC has coordinates equal to equation (3). This is the hard step. One way to attack the problem is by repeatedly projecting links onto planes and using trigonometry to solve for joint angles.

To determine the Wrist Center position, we will need:

*the end-effector positions and rotation transformation matrix*

*the d values (distance of the end-effector from the wrist center) from the DHS table*

![alt text][image7]

We will begin with calculating the first of the three required thetas. As presented in the IK lessions and example (Lesson2) we have a schematic representation as shown below, where the wrist center is projected onto the xo and yo plane. This now makes the calculation of theta1 straightforward. We will use the formula below:

![alt_text][image6]

``` python
theta1 = atan2(WC_0[1], WC_0[0])
```
We will now set q1 to 0, and calculate the O2 position in relation to the base. To compute theta2, we will look at the angle between the z2 and x2 axis

![alt text][image8]

``` python
PO2 = Matrix([[dh[a1]], [0], [dh[d1]]])
# Rotate PO2 w.r.t. Z by theta1
PO2 = rotateZ(theta1)* PO2
```
The next few steps involve primarily solving for the joint angles within the triangle formed by the three points / joints - O2, O3 and O5 (with O4 on the O3 - O5 line)
``` python
# As shown in the base diagram above, dh[a2] is the length between O2 and O3
O2_O3 = dh[a2]

# To calculate the length between the wrist center(O5) and O2
PO2_O5 = O5_0 - PO2
O2_O5 = PO2_O5.norm()

# Distance between O3 and O5/WC = (d4^2 + a3^2) in base diagram 
O3_O5 = np.sqrt(dh[d4]*dh[d4] + dh[a3]*dh[a3]) 

# Offset angle between the Y3 axis line(O3, O5), angleO3_offset in the step 3 diagram above
angleO3_offset = atan2(s[a3], s[d4])   

# angle(O3, O2, O5), q21 in figure.
angle_O2 = np.arccos(float((O2_O3**2 + O2_O5**2 - O3_O5**2)/(2*O2_O3*O2_O5)))

# angle(O2, O3, O5)
angle_O3 = np.arccos(float((O2_O3**2 + O3_O5**2 - O2_O5**2)/(2*O2_O3*O3_O5)))

# Let q1 = theta1 (found above) and 
Z2_Primary = T0_2.subs({q1:theta1}).dot([[0], [0], [1], [0]])
Z2_Rot =  tf.transformations.rotation_matrix(-angle_O2, Z2_Primary)
theta2Direction = Z2_Rot * PO2_O5
X2_Primary = T0_2.subs({q1:theta1, q2:0}).dot([[1], [0], [0], [0]])
theta2 = np.arccos(float(np.dot(X2_Primary, theta2Direction[0:4])))
theta3 = ((pi/2 + angleO3_offset) - angle_O3).evalf()
```
##### Step 4: Once the first three joint variables are known, calculate R0_3 via application of homogeneous transforms up to the WC.

`R0_3` is calculated by multiplying the transformation matrices for `T0_1` through `T2_3`. `R3_6` is calculated in a similar fashion.

```python
R0_3 = T0_3[0:3,0:3]
R3_6 = (T3_4*T4_5*T5_6)[:3,:3]
```

##### Step 5: find a set of Euler angles corresponding to the rotation matrix
