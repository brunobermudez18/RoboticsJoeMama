import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj

# Convert degrees to radians
deg_to_rad = np.pi / 180

# Define the Motoman MH5 robot using DH parameters
mh5 = DHRobot([
    RevoluteDH(d=0.330, a=0.088, alpha=np.pi/2, qlim=[-170 * deg_to_rad, 170 * deg_to_rad]),  # Joint 1
    RevoluteDH(d=0.0, a=0.31, alpha=0, offset=np.pi/2, qlim=[-65 * deg_to_rad, 150 * deg_to_rad]),  # Joint 2
    RevoluteDH(d=0.0, a=0.04, alpha=np.pi/2, qlim=[-136 * deg_to_rad, 255 * deg_to_rad]),  # Joint 3
    RevoluteDH(d=0.305, a=0.0, alpha=np.pi/2, qlim=[-190 * deg_to_rad, 190 * deg_to_rad]),  # Joint 4
    RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2, qlim=[-125 * deg_to_rad, 125 * deg_to_rad]),  # Joint 5
    RevoluteDH(d=0.0865, a=0.0, alpha=0, qlim=[-360 * deg_to_rad, 360 * deg_to_rad]),  # Joint 6
], name="Motoman_MH5", base=SE3(0, 0, 0))

# Define the TCP (Tool Center Point) alignment
mh5.tool = SE3.OA([0, -1, 0], [1, 0, 0])  # X-axis forward, Y-axis backward

# Define the full cube trajectory (all 12 edges)
T = np.array([
    [ 0.55,  0.35,  0.15],  # A (Start)
    [ 0.55, -0.20,  0.15],  # B
    [ 0.70, -0.20,  0.15],  # C
    [ 0.70,  0.35,  0.15],  # D
    [ 0.55,  0.35,  0.15],  # A (Close Bottom Face)

    [ 0.55,  0.35,  0.40],  # E (Move up)
    [ 0.55, -0.20,  0.40],  # F
    [ 0.70, -0.20,  0.40],  # G
    [ 0.70,  0.35,  0.40],  # H
    [ 0.55,  0.35,  0.40],  # E (Close Top Face)

    [ 0.55, -0.20,  0.40],  # F (Connect top to bottom)
    [ 0.55, -0.20,  0.15],  # B
    [ 0.70, -0.20,  0.15],  # C
    [ 0.70, -0.20,  0.40],  # G
    [ 0.70,  0.35,  0.40],  # H
    [ 0.70,  0.35,  0.15],  # D
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
T_tool = SE3.Trans(-0.15, 0, 0) * SE3.Trans(xyz_traj) * SE3.OA([0, -1, 0], [1, 0, 0])

# Compute inverse kinematics
sol = mh5.ikine_LM(T_tool, q0= [0, 0, 0, 0, 0, 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
    mh5.plot(q=sol.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True)
else:
    print("No valid IK solution found!")