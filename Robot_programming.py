# Import all the necessary libraries
import bpy
import numpy as np

Joint0 = bpy.data.objects['3_Link1']
Joint1 = bpy.data.objects['4_Link2']
Joint2 = bpy.data.objects['5_Link3']
ee = bpy.data.objects['6_EndEffector']

# Define the robot parameters
L1 = 0.026236                # Length of link
L2 = 0.46671
L3 = 0.582544

# Trigonometric Functions to be used
def c(theta):
    return np.cos(theta)
def s(theta):
    return np.sin(theta)

# Inverse Kinematics Function

def InverseKinematics(x,y,z):
    d = np.sqrt((0.6+x)**2 + (z-0.21002-L1)**2)
    a = np.arccos((L2**2 + L3**2 - d**2)/(2*L2*L3))

    q3 = np.radians(180)-a
    q2 = np.arctan2(z-0.21002-L1,0.6+x) - np.arctan2(L3*s(q3),L2+L3*c(q3))
    q1 = np.arcsin(y/(L2*c(q2)+L3*c(q3)))
    
    theta1 = q1; theta2 = q2; theta3 = q2 + q3
    theta1 = np.degrees(theta1); theta2 = np.degrees(theta2); theta3 = np.degrees(theta3)
    
    return theta1,theta2,theta3


# Forward Kinematics Function

def ForwardKinematics(q1,q2,q3):
    dh_params = [[0,np.radians(90),L1,q1],[L2,0,0,q2],[L3,0,0,q3]]
    a = [row[0] for row in dh_params]
    alpha = [row[1] for row in dh_params]
    d = [row[2] for row in dh_params]
    theta = [row[3] for row in dh_params]
    
    # Find the Tranformation matrix between Joint 1 and Joint 2
    T1_2 = [[c(theta[0]), -s(theta[0])*c(alpha[0]), s(theta[0])*s(alpha[0]), a[0]*c(theta[0])],
            [s(alpha[0]), c(theta[0])*c(alpha[0]), -c(theta[0])*s(alpha[0]), a[0]*s(theta[0])],
            [0, s(alpha[0]), c(alpha[0]), d[0]],
            [0, 0, 0, 1]]

    # Find the Tranformation matrix between Joint 2 and Joint 3
    T2_3 = [[c(theta[1]), -s(theta[1])*c(alpha[1]), s(theta[1])*s(alpha[1]), a[1]*c(theta[1])],
            [s(alpha[1]), c(theta[1])*c(alpha[1]), -c(theta[1])*s(alpha[1]), a[1]*s(theta[1])],
            [0, s(alpha[1]), c(alpha[1]), d[1]],
            [0, 0, 0, 1]]

    # Find the Tranformation matrix between Joint 3 and Joint 4
    T3_4 = [[c(theta[2]), -s(theta[2])*c(alpha[2]), s(theta[2])*s(alpha[2]), a[2]*c(theta[2])],
            [s(alpha[2]), c(theta[2])*c(alpha[2]), -c(theta[2])*s(alpha[2]), a[2]*s(theta[2])],
            [0, s(alpha[2]), c(alpha[2]), d[2]],
            [0, 0, 0, 1]]
    # Calculating the Transformation Matrix of the Robot from base to end effector
    T_1 = np.dot(T1_2,T2_3)
    T_final = np.dot(T_1,T3_4)

    # Extract the end effector position
    position = T_final[:,3]
    x,y,z,m = position
    
    return x,y,z

# WORKSPACE BOUNDARY
#   z_min = 0.9, z_max = 1.15
#   y_min = -0.8, y_max = 0.8
#   x_min = -0.18, x_max = -0.1

# End Effector Positions
x1,y1,z1 = -0.1,0.8,1.15
x2,y2,z2 = -0.1,0.8,1.1
x3,y3,z3 = -0.1,0.6,1.1
x4,y4,z4 = -0.1,0.4,1.1
x5,y5,z5 = -0.1,0,1.1
x6,y6,z6 = -0.1,-0.3,1.1
x7,y7,z7 = -0.1,-0.3,1

#### POSITION 1
bpy.context.scene.frame_set(0)
q1, q2, q3 = InverseKinematics(x1,y1,z1)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)


#### POSITION 2
bpy.context.scene.frame_set(20)
q1, q2, q3 = InverseKinematics(x2,y2,z2)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)

#### POSITION 3
bpy.context.scene.frame_set(40)
q1, q2, q3 = InverseKinematics(x3,y3,z3)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)

#### POSITION 4
bpy.context.scene.frame_set(60)
q1, q2, q3 = InverseKinematics(x4,y4,z4)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)

#### POSITION 5
bpy.context.scene.frame_set(80)
q1, q2, q3 = InverseKinematics(x5,y5,z5)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)

#### POSITION 6
bpy.context.scene.frame_set(100)
q1, q2, q3 = InverseKinematics(x6,y6,z6)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

Joint0.keyframe_insert(data_path='rotation_euler',index=-1)
Joint1.keyframe_insert(data_path='rotation_euler',index=-1)
Joint2.keyframe_insert(data_path='rotation_euler',index=-1)

#### POSITION 7
bpy.context.scene.frame_set(120)
q1, q2, q3 = InverseKinematics(x7,y7,z7)

theta1_final = q1
theta2_final = np.degrees(np.radians(90) - np.radians(q2))
theta3_final = q3

# Control the robot arm
Joint0.rotation_euler[2] = np.radians(theta1_final)
Joint1.rotation_euler[1] = np.radians(theta2_final)
Joint2.rotation_euler[1] = np.radians(theta3_final)

# Verification
# x_pos, y_pos, z_pos = ForwardKinematics(np.radians(q1),np.radians(q2),np.radians(q3))