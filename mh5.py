import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj, jtraj
from codigoVerTrayect import plot_robot_trajectory

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
T_tool = SE3.Trans(-0.15, 0, 0) * SE3.Trans(xyz_traj) * SE3.OA([0, -1, 0], [1, 0, 0])

# Compute inverse kinematics
sol = mh5.ikine_LM(T_tool, q0=[0, 0, 0, 0, 0, 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
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
    
    # Add angle vs. time plot for sol.q
    dt = 0.02  # From mstraj
    time = np.linspace(0, len(sol.q) * dt, len(sol.q))
    fig, axs = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    for i in range(6):
        axs[i].plot(time, sol.q[:, i], label=f"Joint {i+1}")
        axs[i].set_ylabel(f"Angle (rad)")
        axs[i].grid(True)
        axs[i].legend()
    axs[5].set_xlabel("Time (s)")
    plt.suptitle("Joint Angles vs. Time (sol 1)")
    plt.tight_layout()
    plt.show()
else:
    print("Fuck off!")
    
T_tool2 = SE3(-0.15, 0, 0) 
T_cubo = [T_tool2 * SE3(p[0], p[1], p[2]) for p in T]
    
sol2 = mh5.ikine_LM(SE3(T_cubo), q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0], ilimit=3000, slimit=10, tol=0.000000000001)
if sol2.success:
    print("IK solution found!")
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
    
    # Add angle vs. time plot for sol2.q
    dt = 0.25  # From plot_robot_trajectory
    time = np.linspace(0, len(sol2.q) * dt, len(sol2.q))
    fig, axs = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    for i in range(6):
        axs[i].plot(time, sol2.q[:, i], label=f"Joint {i+1}")
        axs[i].set_ylabel(f"Angle (rad)")
        axs[i].grid(True)
        axs[i].legend()
    axs[5].set_xlabel("Time (s)")
    plt.suptitle("Joint Angles vs. Time (sol 2)")
    plt.tight_layout()
    plt.show()
else:
    print("Fuck off")

# New section: Joint Space Trajectory
# Convert target coordinates from mm to m
targets = np.array([
    [563.48 / 1000, 0.0 / 1000, 350.0 / 1000],      # Joint 1: x=563.48 mm, y=0 mm, z=350 mm
    [623.69 / 1000, 103.92 / 1000, 290.99 / 1000],  # Joint 2: x=623.69 mm, y=103.92 mm, z=290.99 mm
    [685.27 / 1000, 64.18 / 1000, 214.152 / 1000],  # Joint 3: x=685.27 mm, y=64.18 mm, z=214.152 mm
    [626.51 / 1000, -13.2 / 1000, 235.06 / 1000],   # Joint 4: x=626.51 mm, y=-13.2 mm, z=235.06 mm
    [544.37 / 1000, -116.48 / 1000, 291.33 / 1000]  # Joint 5: x=544.37 mm, y=-116.48 mm, z=291.33 mm
])

# Compute initial joint configuration (using first target)
T_tool_start = SE3.Trans(targets[0]) * mh5.tool
q_start_result = mh5.ikine_LM(T_tool_start, q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])
if not q_start_result.success:
    raise ValueError("Initial IK solution failed!")
q_start = q_start_result.q  # Extract the joint configuration

# Compute joint configurations for each target
q_configs = [q_start]
for target in targets[1:]:
    T_tool = SE3.Trans(target) * mh5.tool
    q_result = mh5.ikine_LM(T_tool, q0=q_configs[-1], mask=[1, 1, 1, 0, 0, 0])
    if not q_result.success:
        print(f"Warning: IK failed for target {target}, using previous configuration.")
        q_configs.append(q_configs[-1])
    else:
        q_configs.append(q_result.q)
for i, q in enumerate(q_configs):
    print(f"Configuration {i}: {q}")

q_configs = np.array(q_configs)
q_joints = np.array([[0.      ,   -0.70352504, -0.32423885 , 0.       ,  -1.30962079 , 0.],
                    [0.16213138, -0.98053853, -0.00173898 ,-0.02319557, -1.21496477 , 0.],
                    [0.09208002, -1.28675044,  0.37075343, -0.0117111 , -1.08838462 , 0. ],
                    [-2.11054600e-02 ,-1.08734447e+00 , 5.00307569e-03, -3.04966318e-04 ,-1.21594850e+00,  0.00000000e+00 ],
                    [-0.20711171, -0.81764164, -0.34733692 , 0.02443471, -1.32466853 , 0.]])

# Generate joint space trajectory
total_segments = len(q_configs) - 1
total_time = 3.0  # 3 seconds per segment
points_per_segment = 30
total_points = total_segments * points_per_segment + 1
t = np.linspace(0, total_time * total_segments, total_points)

q_traj = np.zeros((total_points, 6))
for i in range(total_segments):
    q_start = q_joints[i]
    q_end = q_joints[i + 1]
    segment_t = np.linspace(0, total_time, points_per_segment + 1)
    segment_traj = jtraj(q_start, q_end, segment_t).q
    start_idx = i * points_per_segment
    end_idx = start_idx + points_per_segment + 1
    q_traj[start_idx:end_idx] = segment_traj

# Visualize the complete trajectory
if np.any(q_traj):
    print("Joint space trajectory computed!")
    mh5.plot(q=q_traj, limits=[-1, 1, -1, 1, -0.15, 1.5], eeframe=True, shadow=True, jointaxes=False, block=True)
else:
    print("No joint space trajectory due to invalid configurations!")

# Compute end-effector positions for Cartesian trajectory
T_ee = np.array([mh5.fkine(qi).t for qi in q_traj])

# Plot end-effector trajectory in Cartesian space
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(T_ee[:, 0], T_ee[:, 1], T_ee[:, 2], label="End-effector Path")
ax.scatter(T_ee[0, 0], T_ee[0, 1], T_ee[0, 2], color="red", marker="*", label="Start")
ax.scatter(T_ee[-1, 0], T_ee[-1, 1], T_ee[-1, 2], color="blue", marker="o", label="End")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("End-Effector Trajectory in Cartesian Space")
ax.legend()
plt.show()

# Plot joint angles vs. time
fig, axs = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
time = t
for i in range(6):
    axs[i].plot(time, q_traj[:, i], label=f"Joint {i+1}")
    axs[i].set_ylabel(f"Angle (rad)")
    axs[i].grid(True)
    axs[i].legend()
axs[5].set_xlabel("Time (s)")
plt.suptitle("Joint Angles vs. Time")
plt.tight_layout()
plt.show()