import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

# Declare the Motoman MH5 robot using DH parameters
crx = DHRobot([
    RevoluteDH(d=0.0, a=0.13, alpha = -np.pi/2, offset = -np.pi/2, qlim=[-3.49, 3.49]),  # Joint 1: θ1, d=330mm, a=0, α=0°
    RevoluteDH(d=0.0, a=0.41, alpha=0, offset = -np.pi/2, qlim=[-3.14, 3.14]),  # Joint 2: θ2, d=0, a=88mm, α=90°
    RevoluteDH(d=0.0, a=0.13, alpha = 0, offset = -np.pi/2, qlim=[-5.54, 5.54]),  # Joint 3: θ3, d=0, a=310mm, α=0°
    RevoluteDH(d=0.43, a=0.0, alpha = -np.pi/2, offset = np.pi/2, qlim=[-3.31, 3.31]),  # Joint 4: θ4+90°, d=305mm, a=40mm, α=90°
    RevoluteDH(d=0.13, a=0.0, alpha = np.pi/2, qlim=[-3.14, 3.14]),  # Joint 5: θ5, d=0, a=0, α=-90°
    RevoluteDH(d=0.145, a=0.0, alpha = 0, qlim=[-3.93, 3.93]),  # Joint 6: θ6, d=86.5mm, a=0, α=90°
], name="CRX-5iA", base=np.eye(4))
crx.qz = [0, 0, 0, 0, 0, 0]  # Zero angles
# Plot the robot in its zero configuration
crx.plot(crx.qz, block = True)

# Show the plot
plt.show()

# Optional: Print robot information
print(crx)

joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0) 
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

T04DH = crx.fkine([joint1, joint2, joint3, joint4, joint5, joint6])
print(T04DH)