import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj, jtraj
from codigoVerTrayect import plot_robot_trajectory

# Convert degrees to radians
deg_to_rad = np.pi / 180

# Define the Crx5ia robot using DH parameters
mh5 = DHRobot([
    RevoluteDH(d=0.0, a=0.13, alpha=-np.pi/2, offset=-np.pi/2, qlim=[-3.49, 3.49]),  # Joint 1: θ1, d=0mm, a=130mm, α=-90°
    RevoluteDH(d=0.0, a=0.41, alpha=0, offset=-np.pi/2, qlim=[-3.14, 3.14]),  # Joint 2: θ2, d=0, a=410mm, α=0°
    RevoluteDH(d=0.0, a=0.13, alpha=0, offset=-np.pi/2, qlim=[-5.54, 5.54]),  # Joint 3: θ3, d=0, a=130mm, α=0°
    RevoluteDH(d=0.43, a=0.0, alpha=-np.pi/2, offset=np.pi/2, qlim=[-3.31, 3.31]),  # Joint 4: θ4, d=430mm, a=0, α=-90°
    RevoluteDH(d=0.13, a=0.0, alpha=np.pi/2, qlim=[-3.14, 3.14]),  # Joint 5: θ5, d=130mm, a=0, α=90°
    RevoluteDH(d=0.145, a=0.0, alpha=0, qlim=[-3.93, 3.93]),  # Joint 6: θ6, d=145mm, a=0, α=0°
], name="Crx5ia", base=np.eye(4))

# Define the TCP (Tool Center Point) alignment
mh5.tool = SE3.OA([0, 1, 0], [0, 0, 1])  # X-axis forward, Z-axis upward

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
T_tool = SE3.Trans(-0.21, -0.325, 0.125) * SE3.Trans(xyz_traj)

# Compute inverse kinematics
sol = mh5.ikine_LM(T_tool, q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
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
    
T_tool2 = SE3(-0.21, -0.325, 0.125) 
T_cubo = [T_tool2 * SE3(p[0], p[1], p[2]) for p in T]
    
sol2 = mh5.ikine_LM(SE3(T_cubo), q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0], ilimit=3000, slimit=10, tol=0.000000000001)
if sol2.success:
    print("IK solution found!")
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
    [711.941 / 1000, -130.0 / 1000, 410.0 / 1000],      # Joint 1: x=711.941 mm, y=-130 mm, z=410 mm
    [801.957 / 1000, -258.702 / 1000, 329.289 / 1000],  # Joint 2: x=801.957 mm, y=-258.702 mm, z=329.289 mm
    [648.372 / 1000, -369.435 / 1000, 329.289 / 1000],  # Joint 3: x=648.372 mm, y=-369.435 mm, z=329.289 mm
    [535.854 / 1000, -458.551 / 1000, 449.449 / 1000],  # Joint 4: x=535.854 mm, y=-458.551 mm, z=449.449 mm
    [438.122 / 1000, -316.586 / 1000, 507.048 / 1000]   # Joint 5: x=438.122 mm, y=-316.586 mm, z=507.048 mm
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

q_configs = np.array(q_configs)

# Generate joint space trajectory
total_segments = len(q_configs) - 1
total_time = 3.0  # 3 seconds per segment
points_per_segment = 30
total_points = total_segments * points_per_segment + 1
t = np.linspace(0, total_time * total_segments, total_points)

q_traj = np.zeros((total_points, 6))
for i in range(total_segments):
    q_start = q_configs[i]
    q_end = q_configs[i + 1]
    segment_t = np.linspace(0, total_time, points_per_segment + 1)
    segment_traj = jtraj(q_start, q_end, segment_t).q
    start_idx = i * points_per_segment
    end_idx = start_idx + points_per_segment + 1
    q_traj[start_idx:end_idx] = segment_traj

# Visualize the complete trajectory
if np.any(q_traj):
    print("Joint space trajectory computed!")
    mh5.plot(q=q_traj, limits=[-1, 1, -1, 1, -0.21, 1.5], eeframe=True, shadow=True, jointaxes=False, block=True)
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