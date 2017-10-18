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

The last three joints in a manipulator are revolute joints and those intersect at a single point, namely it is called so call spherical wrist. So we can think the Cartesian coordinates of the wrist center first, and next the composition of rotations to orient the end effector. the equations are in the IK_server.py line No.53- def IK_parameter.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

I could complete 8/10 pick and place cycles by the code in the IK_server.py. In the IK_server.py, with reference to the video provided in the project,

* At first I created symbols for DH_parameters and define the Table, and rotation matrix. 
* Then I define Inverse kinematics equations in IK_parameter function.
* Also, I checked with IK_debug.py about the IK calculation error. I iterated several times changing parameters and checking the error, and simulation. 

I guess in the practical situation, we need to consider other factors such as the distortion of arm by its action or the weight of the load, ..etc.


example image:
![alt text][image4]


