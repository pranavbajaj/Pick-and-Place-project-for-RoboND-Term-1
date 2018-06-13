## Project: Kinematics Pick & Place
---
[image1]: ./misc_images/DH_table.jpg
[image2]: ./misc_images/joint_tanformation_matrix.png
[image3]: ./misc_images/homo_trans_base_gripper.jpg 
[image4]: ./misc_images/theta1.jpg 
[image5]: ./misc_images/tri.png
[image11]: ./misc_images/theta2andtheta3.jpg 
[image6]: ./misc_images/R3_6.png
[image7]: ./misc_images/misc2.png
[image8]: ./misc_images/Capture1.png
[image9]: ./misc_images/Capture2.png
[image10]: ./misc_images/Capture4.png

### Kinematic Analysis
#### 1.Notation Figure of Robot and its DH table 

![alt text][image1]

##### To obtain DH table following setps should taken
##### Step1: Find the coordinates of each joint from ~/RoboND-Kinematics-Project/kuka_arm/urdf/kr210.urdf.xacro file 
##### Step2: A. Label all joints form 1 to 6
#####        B. Label all link from 0 to 6
#####        C. Draw lines through all joints, defining the joint axis. 
#####        D. Assingn the Z-axis and X-axis of each from to point along its joint axis. 
##### Step3: By following the following rules find the value of all the DH parameters for all the frames. 
#####        A. alpha(i-1): angle between Zi-1 and Zi measured about Xi-1 in a right-hand sense.
#####        B. a(i-1): distance from zi-1 to Zi measured along Xi-1 where Xi-1 is perpendicular to both Zi-1 and Zi.
#####        C. d(i): signed distance from Xi-1 to Xi measured along Zi.
#####        D. q(i): angle between Xi-1 to Xi measured about Zi in a right-hand sense. Variable quentity in case of revolute joint.


#### 2. Transformation Matrix about each joint is (TX-Y means between link X and Y)

![alt text][image2]

#### 3. Homogenous Transformation Matrix between base link and the gripper using only the position and orientation of gripper 

![alt text][image3]

#### 4. Breaking of Inverse Kinametic Problem:
##### step 1: The Homogenous Transformation Matrix we obtain above is the Transformation Matrix with respect to base link frame. Inorder to obtain the Homgenous Transformation Matrix with respect to DH frame of gripper we Multiply it by the correction Matrix.
##### step 2: We divide the problem of finding joint angles into two simpler solution, one is to finding cartesian coordinate of wrist center and then the composition of rotation to orient the end effector. Wrist center can be found by subtracting the respective offset between the wrist center and the gripper. Offset is equale to d7. From the Wrist center coodinates we can obtain the value of q1,q2 and q3.  
###### As q1 is responsibile for the postion of x and y coordinates of wrist center and has no influence on z coordinate 
![alt text][image4] 
###### q2 and q3 can be obtained by following method
![alt text][image5]
![alt text][image11]

###### By observing the rotation matrix between joint3 and joint6(R3_6), we can find q4, q5 and q6. 
![alt text][image6]

### Project Implementation

#### Following results were obtained 
##### Enviroment setup
![alt text][image7] 
##### Moving toward the target object 
![alt text][image8]
##### Grabing the target object 
![alt text][image9]
##### Moved to drop off loaction
![alt text][image10]





