#python program for inverse and forward kinematics
import bpy
import numpy as np

#Set the mode to pose 
bpy.ops.object.mode_set(mode='POSE')

# get specific bone name 'Bone'
End_eff = bpy.data.objects['Armature'].pose.bones["Bone.003"]

#Trajectory Planning using cubic polynomial
def cubic_coefficients(q0, qf, t0, tf):
    a0 = q0
    a1 = 0
    a2 = (3 * (qf - q0) / ((tf - t0) ** 2))
    a3 = (-2 * (qf - q0) / ((tf - t0) ** 3))
    return a0, a1, a2, a3

def cubic_trajectory(t, a0, a1, a2, a3):
    return a0 + a1 * t + a2 * t**2 + a3 * t**3

#Initializing an array to store the robot trajectory
x_traj = np.zeros(1)
y_traj = np.zeros(1)
z_traj = np.zeros(1)
tframe = np.zeros(1)

#Start generating trajectory in every frame
start_frame = 1
last_frame = 240

#A loop to generate position equal to the number of frames
t = np.linspace(start_frame-1,last_frame,240)
for i in t:
    t0 = start_frame
    tf = last_frame
    tframe += start_frame
    End_eff_home = (0,0,0)
    End_eff_final = (0,0,0)
    #X coordinates for the trajectory
    q0_xi, q0_xf = End_eff_home[0], End_eff_final[0]
    x0,x1,x2,x3 = cubic_coefficients(q0_xi,q0_xf,t0,tf)
    x_traj= cubic_trajectory(tframe,x0,x1,x2,x3)
    
    #y coordinates for the trajectory
    q0_yi, q0_yf = End_eff_home[1], End_eff_final[1]
    y0,y1,y2,y3 = cubic_coefficients(q0_yi,q0_yf,t0,tf)
    y_traj= cubic_trajectory(tframe,y0,y1,y2,y3)
    
    #z coordinates for the trajectory
    q0_zi, q0_zf = End_eff_home[2], End_eff_final[2]
    z0,z1,z2,z3 = cubic_coefficients(q0_zi,q0_zf,t0,tf)
    z_traj= cubic_trajectory(tframe,z0,z1,z2,z3)
    
    
    End_eff.keyframe_insert("location", frame = i)

    End_eff.location = (x_traj,y_traj,z_traj)

    End_eff.keyframe_insert("location", frame = last_frame)
#robot_pos.location.y =3