import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj, jtraj
from codigoVerTrayect import plot_robot_trajectory

# Convert degrees to radians
deg_to_rad = np.pi / 180

# Define the Motoman MH5 robot using DH parameters
lrmate = DHRobot([
    RevoluteDH(d=0.33, a=0.05, alpha=-np.pi/2, qlim=[-2.97, 2.97]),  # Joint 1: θ1, d=330mm, a=0, α=0°
    RevoluteDH(d=0.0, a=0.33, alpha=0, offset=-np.pi/2, qlim=[-1.13, 2.62]),  # Joint 2: θ2, d=0, a=88mm, α=90°
    RevoluteDH(d=0.0, a=0.035, alpha=-np.pi/2, qlim=[-2.37, 4.45]),  # Joint 3: θ3, d=0, a=310mm, α=0°
    RevoluteDH(d=0.335, a=0.0, alpha=np.pi/2, qlim=[-3.32, 3.32]),  # Joint 4: θ4+90°, d=305mm, a=40mm, α=90°
    RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2, qlim=[-2.18, 2.18]),  # Joint 5: θ5, d=0, a=0, α=-90°
    RevoluteDH(d=0.08, a=0.0, alpha=0, qlim=[-6.28, 6.28]),  # Joint 6: θ6, d=86.5mm, a=0, α=90°
], name="lrmate200ib", base=np.eye(4))

# Define the TCP (Tool Center Point) alignment
lrmate.tool = SE3.OA([0, -1, 0], [1, 0, 0])  # X-axis forward, Y-axis backward

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
sol = lrmate.ikine_LM(T_tool, q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])

# Check if solution is valid
if sol.success:
    print("IK solution found!")
    #lrmate.plot(q=sol.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True)
    p_lim=[-1, 1, -1, 1, -0.15, 1.5]
    plot_robot_trajectory(
        robot=lrmate,
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
    
sol2 = lrmate.ikine_LM(SE3(T_cubo), q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0], ilimit=3000, slimit=10, tol=0.000000000001)
if sol2.success:
    print("IK solution found!")
    #lrmate.plot(q=sol2.q, limits=[-0.3, 0.8, -0.6, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=False, block=True, dt=0.25)
    p_lim=[-1, 1, -1, 1, -0.15, 1.5]
    plot_robot_trajectory(
        robot=lrmate,
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
    [0.63569, 0.00000, 0.36500],  # Target 1
    [0.68865, 0.13971, 0.30421],  # Target 2
    [0.62794, 0.08435, 0.24038],  # Target 3
    [0.68589, -0.03295, 0.31632], # Target 4
    [0.56899, -0.03295, 0.34820]  # Target 5
])

# Compute initial joint configuration (using first target)
T_tool_start = SE3.Trans(targets[0]) * lrmate.tool
q_start_result = lrmate.ikine_LM(T_tool_start, q0=[0, 0, 0, 0, np.deg2rad(-80), 0], mask=[1, 1, 1, 0, 0, 0])
if not q_start_result.success:
    raise ValueError("Initial IK solution failed!")
q_start = q_start_result.q  # Extract the joint configuration

# Compute joint configurations for each target
q_configs = [q_start]
for target in targets[1:]:
    T_tool = SE3.Trans(target) * lrmate.tool
    q_result = lrmate.ikine_LM(T_tool, q0=q_configs[-1], mask=[1, 1, 1, 0, 0, 0])
    if not q_result.success:
        print(f"Warning: IK failed for target {target}, using previous configuration.")
        q_configs.append(q_configs[-1])
    else:
        q_configs.append(q_result.q)
for i, q in enumerate(q_configs):
    print(f"Configuration {i}: {q}")

q_configs = np.array(q_configs)
q_joints = np.array([[ 0.       ,   0.86925924 ,-0.08280846 , 0.    ,     -1.2313281 ,  0.],
                    [0.19789615 , 1.15307221, -0.44613014, -0.02219678, -1.10923986 , 0.],
                    [0.13148087 , 1.08651305, -0.09789314, -0.01735719, -1.20680905 , 0.],
                    [-0.04756592 , 1.08902083, -0.35082488,  0.00409072, -1.16291317, 0.],
                    [-0.05704701 , 0.75072145 , 0.20815861 , 0.0058179,  -1.35699313 , 0.]])

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
    lrmate.plot(q=q_traj, limits=[-1, 1, -1, 1, -0.15, 1.5], eeframe=True, shadow=True, jointaxes=False, block=True)
else:
    print("No joint space trajectory due to invalid configurations!")

# Compute end-effector positions for Cartesian trajectory
T_ee = np.array([lrmate.fkine(qi).t for qi in q_traj])

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