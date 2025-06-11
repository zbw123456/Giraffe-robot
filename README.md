# Giraffe-robot
A typical project for Intro to Robotics

1. Introduction
This project presents the design, simulation, and control of a ceiling-mounted robotic arm (“giraffe robot”) to automate microphone delivery to audience members in a conference room during Q&A sessions. The robot is capable of reaching a target location within a 5 × 12 m area with a desired orientation (30° pitch), enhancing efficiency and interactivity in public talks.

2. System Specifications
2.1 Workspace and Mounting
Room height: 4 m

Task space: 5 m × 12 m horizontal area, down to 1 m height

Mounting: Ceiling, center of the room

2.2 Degrees of Freedom
5 DOF Robot with:

2 DoF spherical joint at base (intersecting revolute joints)

1 prismatic joint for linear extension

2 revolute joints for microphone orientation

3. URDF Model
Using standard URDF syntax, the robot is modeled with:

Links: Base, arm, extension, wrist1, wrist2, microphone

Joints:

joint1: Revolute (yaw)

joint2: Revolute (pitch)

joint3: Prismatic (extension)

joint4: Revolute (mic pitch)

joint5: Revolute (mic roll)

The coordinate frames are placed to match Figure 2 (side view) and Figure 3 (top view) from the project brief.

4. Kinematics
4.1 Forward Kinematics
Using transformation matrices or via Pinocchio, the transformation from the base to the end-effector frame is computed:

T
end-effector
=
T
1
⋅
T
2
⋅
T
3
⋅
T
4
⋅
T
5
T 
end-effector
​
 =T 
1
​
 ⋅T 
2
​
 ⋅T 
3
​
 ⋅T 
4
​
 ⋅T 
5
​
 
4.2 Differential Kinematics
Jacobian matrix J is computed via Pinocchio for mapping joint velocities to end-effector velocities:

x
˙
=
J
(
q
)
q
˙
x
˙
 =J(q) 
q
˙
​
 
This is used for control and null-space projection.

5. Dynamics Simulation
Using Pinocchio’s RNEA (Recursive Newton-Euler Algorithm), we simulate the dynamics:

τ
=
M
(
q
)
q
¨
+
C
(
q
,
q
˙
)
q
˙
+
g
(
q
)
τ=M(q) 
q
¨
​
 +C(q, 
q
˙
​
 ) 
q
˙
​
 +g(q)
