import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj
from codigoVerTrayect import plot_robot_trajectory

# Convert degrees to radians
deg_to_rad = np.pi / 180

# Define the Motoman MH5 robot using DH parameters
mh5 = DHRobot([
    RevoluteDH(d=0.0, a=0.13, alpha = -np.pi/2, offset = -np.pi/2, qlim=[-3.49, 3.49]),  # Joint 1: θ1, d=330mm, a=0, α=0°
    RevoluteDH(d=0.0, a=0.41, alpha=0, offset = -np.pi/2, qlim=[-3.14, 3.14]),  # Joint 2: θ2, d=0, a=88mm, α=90°
    RevoluteDH(d=0.0, a=0.13, alpha = 0, offset = -np.pi/2, qlim=[-5.54, 5.54]),  # Joint 3: θ3, d=0, a=310mm, α=0°
    RevoluteDH(d=0.43, a=0.0, alpha = -np.pi/2, offset = np.pi/2, qlim=[-3.31, 3.31]),  # Joint 4: θ4+90°, d=305mm, a=40mm, α=90°
    RevoluteDH(d=0.13, a=0.0, alpha = np.pi/2, qlim=[-3.14, 3.14]),  # Joint 5: θ5, d=0, a=0, α=-90°
    RevoluteDH(d=0.145, a=0.0, alpha = 0, qlim=[-3.93, 3.93]),  # Joint 6: θ6, d=86.5mm, a=0, α=90°
], name="Crx5ia", base=np.eye(4))

mh5.qz = [0, 0, 0, 0, 0, 0]  # Zero angles
#mh5.plot(mh5.qz, block = True)

# Define the TCP (Tool Center Point) alignment
mh5.tool = SE3.OA([0, 1, 0], [0, 0, 1])  # X-axis forward, Y-axis backward
#mh5.teach([0, 0, 0, 0, 0, 0])
# Define the full cube trajectory (all 12 edges)
T = np.array([
    [0.55,  0.00,  0.20],  # A (Start)
    [0.55,  0.15,  0.20],  # B
    [0.70,  0.15,  0.20],  # C
    [0.70,  0.00,  0.20],  # D
    [0.55,  0.00,  0.20],  # A (Close Bottom Face)

    [0.55,  0.00,  0.35],  # E (Move up)
    [0.55,  0.15,  0.35],  # F
    [0.70,  0.15,  0.35],  # G
    [0.70,  0.00,  0.35],  # H
    [0.55,  0.00,  0.35],  # E (Close Top Face)

    [0.55,  0.15,  0.35],  # F (Connect top to bottom)
    [0.55,  0.15,  0.20],  # B
    [0.70,  0.15,  0.20],  # C
    [0.70,  0.15,  0.35],  # G
    [0.70,  0.00,  0.35],  # H
    [0.70,  0.00,  0.20]   # D
])

# Generate a smooth trajectory using mstraj
via = T.copy()  # Store waypoints efficiently
xyz_traj = mstraj(via, qdmax=[0.5, 0.5, 0.5], dt=0.02, tacc=0.2).q

# Plot the generated trajectory (3D visualization)
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
plt.plot(xyz_traj[:, 0], xyz_traj[:, 1], xyz_traj[:, 2], color="red")
ax.scatter(xyz_traj[0, 0], xyz_traj[0, 1], xyz_traj[0, 2], color="red", marker="*")  # Start point
ax.scatter(xyz_traj[-1, 0], xyz_traj[-1, 1], xyz_traj[-1, 2], color="blue", marker="o")  # End point
plt.show()

# Define T_tool for inverse kinematics
T_tool = SE3.Trans(-0.21, -0.325, 0.125) * SE3.Trans(xyz_traj) #* SE3.OA([0, 1, 0], [1, 0, 0])

# Compute inverse kinematics
sol = mh5.ikine_LM(T_tool, q0= [0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
    #mh5.plot(q=sol.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True)
    p_lim=[-1, 1, -1, 1, -0.21, 1.5]
    plot_robot_trajectory(
        robot=mh5,
        q_trajectory=sol.q,
        limits=p_lim,
        eeframe=True,
        jointaxes=False,
        shadow=True,
        drawing_mode='continuous',  # o 'segments' si prefieres
        traj_color='r',             # Color de la trayectoria completa
        drawing_color='b',          # Color del trazo principal
        dt=0.05,
        block=True
    )
else:
    print("Fuck off!")
    
T_tool2 = SE3(-0.21, -0.325, 0.125) 
T_cubo  = [T_tool2 * SE3(p[0], p[1], p[2]) for p in T]
    
sol2 = mh5.ikine_LM(SE3(T_cubo), q0= [0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0], ilimit = 3000, slimit = 10, tol = 0.000000000001)
if sol2.success:
    print("IK solution found!")
    #mh5.plot(q=sol2.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True, dt = 0.25)
    p_lim=[-1, 1, -1, 1, -0.21, 1.5]
    plot_robot_trajectory(
        robot=mh5,
        q_trajectory=sol2.q,
        limits=p_lim,
        eeframe=True,
        jointaxes=False,
        shadow=True,
        drawing_mode='continuous',  # o 'segments' si prefieres
        traj_color='r',             # Color de la trayectoria completa
        drawing_color='b',          # Color del trazo principal
        dt=0.25,
        block=True
    )
else:
    print("Fuck off") 