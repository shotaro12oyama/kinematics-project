## Project: Kinematics Pick & Place

---
**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./DH_parameter.png
[image2]: ./DH_parameter2.png
[image3]: ./wc.png
[image4]: ./result.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


With reference to the lesson video and rviz window opend by the command "roslaunch kuka_arm forward_kinematics.launch", I made DH parameters as follows. 
##### DH_parameters
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | theta1
1->2  | -90        | 0.35   | 0      | theta2-90
2->3  | 0          | 1.25   | 0      | theta3
3->4  | -90        | -0.054 | 1.5    | theta4
4->5  | 90         | 0      | 0      | theta5
5->6  | -90        | 0      | 0      | theta6
6->EE | 0          | 0      | 0.303  | 0

##### link and each parameters location
![alt text][image1]

##### parameter source (highlighted by Red line)
![alt text][image2]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


Link   | transformation matrices (to make it simple, set each theta = 0)
---    | ---
T0_1 = | [[1.0,   0,   0,     0],[  0,  1.0,   0,    0],[  0,   0, 1.0,  0.75],[  0,   0,   0,  1.0]]
T0_2 = | [[  0, 1.0,   0,  0.35],[  0,    0, 1.0,    0],[1.0,   0,   0,  0.75],[  0,   0,   0,  1.0]]
T0_3 = | [[  0, 1.0,   0,  0.35],[  0,    0, 1.0,    0],[1.0,   0,   0,   2.0],[  0,   0,   0,  1.0]]
T0_4 = | [[  0,  0,  1.0,  1.85],[  0, -1.0,   0,    0],[1.0,   0,   0, 1.946],[  0,   0,   0,  1.0]]
T0_5 = | [[  0, 1.0,   0,  1.85],[  0,    0, 1.0,    0],[1.0,   0,   0, 1.946],[  0,   0,   0,  1.0]]
T0_6 = | [[  0,   0, 1.0, 2.153],[  0, -1.0,   0,    0],[1.0,   0,   0, 1.946],[  0,   0,   0,  1.0]]
T0_EE= | [[  0,   0, 1.0, 2.153],[  0, -1.0,   0,    0],[1.0,   0,   0, 1.946],[  0,   0,   0,  1.0]]



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last three joints in a manipulator are revolute joints and those intersect at a single point, namely it is called so call spherical wrist. So we can think the Cartesian coordinates of the wrist center first, and next the composition of rotations to orient the end effector. 

##### equations

* Wrist Center = End Effector (value is from Ros jointtrajectorypoint)  - (0.303) * Rotation Matrix (DH parameter of Link6->EE above) 

* theta1 = atan2(Wrist Center(Y_axis), Wrist Center(X_axis))
* 
** theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.transpose() * ROT_EE
    # Eular angles from rotation matrix
    # More information can be found in the Eular Angles from a Rotation Matrix section
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    theta = [theta1, theta2, theta3, theta4, theta5, theta6]



![alt text][image3]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


## create symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angle symbols

# More information can be found in KR210 Forward Kinematics section
DH_Table = {    alpha0:      0, a0:      0,   d1:  0.75,   q1:         q1,
                alpha1:  -pi/2, a1:   0.35,   d2:     0,   q2: -pi/2 + q2,
                alpha2:      0, a2:   1.25,   d3:     0,   q3:         q3,
                alpha3:  -pi/2, a3: -0.054,   d4:   1.5,   q4:         q4,
                alpha4:   pi/2, a4:      0,   d5:     0,   q5:         q5,
                alpha5:  -pi/2, a5:      0,   d6:     0,   q6:         q6,
                alpha6:      0, a6:      0,   d7: 0.303,   q7:          0,}

# Define Modified DH Tranformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[  cos(q),           -sin(q),           0,             a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                0,                 0,           0,             1]])
    return TF

# Create individual transformation matrices
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

def IK_parameter(px, py, pz, roll, pitch, yaw):
    # Find EE rotation matrix
    # Define RPY rotation matices
    r, p, y = symbols('r p y')
    ROT_x = Matrix([[1,       0,       0],
                    [0,  cos(r), -sin(r)],
                    [0,  sin(r),  cos(r)]])  # ROLL
    ROT_y = Matrix([[cos(p),  0,  sin(p)],
                    [0,       1,       0],
                    [-sin(p), 0,  cos(p)]])  # PITCH
    ROT_z = Matrix([[cos(y), -sin(y),  0],
                    [sin(y),  cos(y),  0],
                    [0,       0,       1]])  # YAW
    ROT_EE = ROT_z * ROT_y * ROT_x
    # More information can be found in KR210 Forward Kinematics section
    Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
    ROT_EE = ROT_EE * Rot_Error
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
    EE = Matrix([[px],
                 [py],
                 [pz]])
    WC = EE - (0.303) * ROT_EE[:, 2]

    # Calculate joint angles using Geomatric IK method
    theta1 = atan2(WC[1], WC[0])
    # SSS triangle for theta2 and theta3
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35),2) + pow((WC[2] - 0.75), 2))
    side_c = 1.25

    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.transpose() * ROT_EE
    # Eular angles from rotation matrix
    # More information can be found in the Eular Angles from a Rotation Matrix section
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    theta = [theta1, theta2, theta3, theta4, theta5, theta6]
    print(theta)
    return theta


example image:
![alt text][image4]


