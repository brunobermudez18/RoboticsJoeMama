import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj
from codigoVerTrayect import plot_robot_trajectory

# Convert degrees to radians
deg_to_rad = np.pi / 180

# Define the Motoman MH5 robot using DH parameters
mh5 = DHRobot([
    RevoluteDH(d=0.33, a=0.05, alpha=-np.pi/2, qlim=[-2.97, 2.97]),  # Joint 1: θ1, d=330mm, a=0, α=0°
    RevoluteDH(d=0.0, a=0.33, alpha=0, offset = -np.pi/2, qlim=[-1.13, 2.62]),  # Joint 2: θ2, d=0, a=88mm, α=90°
    RevoluteDH(d=0.0, a=0.035, alpha=-np.pi/2, qlim=[-2.37, 4.45]),  # Joint 3: θ3, d=0, a=310mm, α=0°
    RevoluteDH(d=0.335, a=0.0, alpha=np.pi/2, qlim=[-3.32, 3.32]),  # Joint 4: θ4+90°, d=305mm, a=40mm, α=90°
    RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2, qlim=[-2.18, 2.18]),  # Joint 5: θ5, d=0, a=0, α=-90°
    RevoluteDH(d=0.08, a=0.0, alpha=0, qlim=[-6.28, 6.28]),  # Joint 6: θ6, d=86.5mm, a=0, α=90°
], name="lrmate200ib", base=np.eye(4))

# Define the TCP (Tool Center Point) alignment
mh5.tool = SE3.OA([0, -1, 0], [1, 0, 0])  # X-axis forward, Y-axis backward

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
T_tool = SE3.Trans(-0.15, 0, 0) * SE3.Trans(xyz_traj) #* SE3.OA([0, -1, 0], [1, 0, 0])

# Compute inverse kinematics
sol = mh5.ikine_LM(T_tool, q0= [0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
    #mh5.plot(q=sol.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True)
    p_lim=[-1, 1, -1, 1, -0.15, 1.5]
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
    
T_tool2 = SE3(-0.15, 0, 0) 
T_cubo  = [T_tool2 * SE3(p[0], p[1], p[2]) for p in T]
    
sol2 = mh5.ikine_LM(SE3(T_cubo), q0= [0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0], ilimit = 3000, slimit = 10, tol = 0.000000000001)
if sol2.success:
    print("IK solution found!")
    #mh5.plot(q=sol2.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True, dt = 0.25)
    p_lim=[-1, 1, -1, 1, -0.15, 1.5]
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