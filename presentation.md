# Project Name:  Robot Simulator with Blender and Python

## Contents
1. **Introduction**
2. **3D Model**
3. **Rigging**
4. **Visualization**
5. **Scripting**
      - Inverse Kinematics
6. **Conclusion**
7. **Future Works**
## Introduction
This project aims to develop an open-source, real-time robot arm simulator inspired by This project aims to develop an open-source, real-time robot arm simulator inspired by the KUKA KR 700 PA palletizing robot. 
![3D Model](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/Picture1.png?raw=true)
 
 ## Objectives
1. 3D Modeling: Create a detailed 3D model of the robotic arm and its components in Blender, inspired by the KUKA KR 700 PA palletizing robot.

2. Rigging: Rig the robot arm in Blender, creating a control system that corresponds to the joints and actuators of the real robot.

3. Scripting: Develop scripts using Python.

## Methodology
There are various steps involved in creating the Simulator. These steps include 3D Modeling, Rigging, Scripting, Creating a user inteface and finally digital twinning the physical robot with the virtual model.

### 1. 3D Modeling
Involves creating a three dimensional representation of the robot using Blender software. The design includes 3 links, 3 joints and an end effector.

![3D Model](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/Picture5.png?raw=true)


### 2. Rigging
Involves adding control to the model. It defines the range of movement for a model by defining  its actions and movements. Rigging in blender inlvoves the use of armatures that are parented to the robot links and constrained to produce a movement similar to that of a real robot.

![3D Model](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/Rigging.png?raw=true)

### Visualization and animation
![Visualization](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/visualization.gif?raw=true)

### 3. Scripting
This is the programming part of the simulator which involves designing a forward kinematics algorithm and an inverse kinematics algorithm.

*Inverse Kinematics and real time simulations*
![3D Model](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/Picture4.png?raw=true)

![Video](https://github.com/CK-Ndiritu/CK-Ndiritu/blob/main/end%20effectors%20move%20from%20home%20to%20final%20position.gif?raw=true)


## 4. Conclusion
Using the python program one can simulate the 2R robot movements and when interfaced with blender open source software its possible to animate, visualize and also verify the simulation results, as what would happen in real situations.

## 5. Extra Work to Work on.
Modeling a more realistic and aesthetic Robot Simulator model in Blender making use of rendering and realistic dimensions.
Creating  a Graphics User Interface in blender or in the python Code to enable a simple interaction of the code and the model by the learner.
Documenting the Project 
Extracting Tutorials from the project for new learners with interest in similar related projects.

*** 
*Refer to the **https://github.com/SiliconWit/real-time-robot-arm-simulator/blob/CK-Ndiritu/RIGGED%203D%20model%20of%20a%202R%20robotic%20arm.blend** for the 3D Model and **https://github.com/SiliconWit/real-time-robot-arm-simulator/blob/CK-Ndiritu/Python_program_for_2R_Robot.py** for the robot control python script.*
***