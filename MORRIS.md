# Blending Reality: Building a Robot Simulator with Blender and Python
## Contents
1. **Introduction**
2. **Methodology**
3. **3D Modeling**
4. **Rigging**
5. **Scripting**
   - Forward Kinematics
   - Inverse Kinematics
6. **Conclusion**
7. **Future Works**
## Introduction
The project aim is to create a virtual representation of a robotic arm using a blender and python. Users can visualize, interact with, and control the robotic arm through the simulator interface. The 3D Model created is inspired by the KUKA KR 700 PA robot.

## Methodology
There are various steps involved in creating the Simulator. These steps include 3D Modeling, Rigging, Scripting, Creating a user inteface and finally digital twinning the physical robot with the virtual model.

![Block Diagram](https://github.com/Mo-rr-is/Robot-Simulator/blob/main/1.jpg?raw=true)

### 1. 3D Modeling
Involves creating a three dimensional representation of the robot using Blender software. The design includes 3 links, 3 joints and an end effector.

![3D Model](https://github.com/Mo-rr-is/Robot-Simulator/blob/main/2.jpg?raw=true)

### 2. Rigging
Involves adding control to the model. It defines the range of movement for a model by defining  its actions and movements. Rigging in blender inlvoves the use of armatures that are parented to the robot links and constrained to produce a movement similar to that of a real robot.

![3D Model Rig](https://github.com/Mo-rr-is/Robot-Simulator/blob/main/3.jpg?raw=true)

### 3. Scripting
This is the programming part of the simulator which involves designing a forward kinematics algorithm and an inverse kinematics algorithm.

*Forward Kinematics*

Involves finding the end effector position of the robot given the joint angles. It is essential in our simulator to verify  if the robot arm has reached the desired position. Creating this algorithm involves various steps. Firstly is the drawing of a Kinematic diagram while following the Denavit Hartenberg rules. The second step is to generate a DH parameters table and lastly generate the transformation matrix of consecutive frames and the final transformation matrix of the base frame to the end effector.

![Kinematic Diagram](https://github.com/Mo-rr-is/Robot-Simulator/blob/main/4.jpg?raw=true)

*Inverse Kinematics*

Involves calculating the Joint angles of the robot arm needed to achieve a desired end effector position. Given an end effector position (x,y,z) then we can find the joint angles using the equations given.

![Inverse Kinematics equations](https://github.com/Mo-rr-is/Robot-Simulator/blob/main/5.jpg?raw=true)

### 4. Conclusion
Through the creation of this simulator we are able to mirror the behavior of the physical robot given certain constraints. It also enables us to interact with and control the robot arm  with much ease, and provides a much better platform for us to test our robot.

### 5. Future Works
- Refining the Kinematics algorithms.
- Error checking to ensure the arm does not extend out of the workspace boundary.
- Adding a Trajectory planning algorithm.
- Adding obstacle avoidance phenomenon and Joint speed controls.
- Picking and placing objects in the simulator.
- Creating a User Interface.
- Real time communication with the physical robot.
*** 
*Refer to the **RobotArm Design.blend file** for the 3D Model and **Robot_programming.py file** for the robot control python script.*
***