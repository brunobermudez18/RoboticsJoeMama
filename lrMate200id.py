import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

# Declare the Motoman MH5 robot using DH parameters
lr = DHRobot([
    RevoluteDH(d=0.33, a=0.05, alpha=-np.pi/2, qlim=[-2.97, 2.97]),  # Joint 1: θ1, d=330mm, a=0, α=0°
    RevoluteDH(d=0.0, a=0.33, alpha=0, offset = -np.pi/2, qlim=[-1.13, 2.62]),  # Joint 2: θ2, d=0, a=88mm, α=90°
    RevoluteDH(d=0.0, a=0.035, alpha=-np.pi/2, qlim=[-2.37, 4.45]),  # Joint 3: θ3, d=0, a=310mm, α=0°
    RevoluteDH(d=0.335, a=0.0, alpha=np.pi/2, qlim=[-3.32, 3.32]),  # Joint 4: θ4+90°, d=305mm, a=40mm, α=90°
    RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2, qlim=[-2.18, 2.18]),  # Joint 5: θ5, d=0, a=0, α=-90°
    RevoluteDH(d=0.08, a=0.0, alpha=0, qlim=[-6.28, 6.28]),  # Joint 6: θ6, d=86.5mm, a=0, α=90°
], name="LR Mate 200 id", base=np.eye(4))
lr.qz = [0, 0, 0, 0, 0, 0]  # Zero angles
# Plot the robot in its zero configuration
lr.plot(lr.qz, block = True)

# Show the plot
plt.show()

# Optional: Print robot information
print(lr)

joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0) 
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

T04DH = lr.fkine([joint1, joint2, joint3, joint4, joint5, joint6])
print(T04DH)