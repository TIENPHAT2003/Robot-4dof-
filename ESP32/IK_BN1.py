import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the robot parameters
H = 80  
D1 = 176 
L1 = 91
L2 = 122
L3 = 78
L4 = 79

# Inverse Kinematics Function (adjusted for the provided formula)
def inverse_kinematics(Px_IK, Py_IK, Pz_IK, Theta_IK):

    t_rad = Theta_IK * (math.pi / 180)
    k = math.sqrt(math.pow(Px_IK, 2) + math.pow(Py_IK, 2))
    
    # Tính toán theta1
    theta1_IK_rad = math.atan2(Py_IK / k, Px_IK / k)
    Theta1_IK = math.degrees(theta1_IK_rad)
    Theta1_IK = round(Theta1_IK)
    
    if Theta1_IK < -180:
        Theta1_IK += 360
    elif Theta1_IK > 180:
        Theta1_IK -= 360
    
    # Tính toán các giá trị trung gian cho theta2 và theta3
    E = Px_IK * math.cos(theta1_IK_rad) + Py_IK * math.sin(theta1_IK_rad) - L1 - L4 * math.cos(t_rad)
    F = Pz_IK - D1 - H - L4 * math.sin(t_rad)
    
    a = -2 * L2 * F
    b = -2 * L2 * E
    d = math.pow(L3, 2) - math.pow(E, 2) - math.pow(F, 2) - math.pow(L2, 2)
    f = math.sqrt(math.pow(a, 2) + math.pow(b, 2))
    alpha = math.atan2((-2 * L2 * F) / f, (-2 * L2 * E) / f)
    
    var_temp = math.pow(d, 2) / math.pow(f, 2)
    if var_temp > 1:
        var_temp = 1
    
    # Tính toán theta2
    theta2_IK_rad = math.atan2((math.sqrt(1 - var_temp)), d / f) + alpha
    Theta2_IK = math.degrees(theta2_IK_rad)
    Theta2_IK = round(Theta2_IK)
    
    if Theta2_IK < -180:
        Theta2_IK += 360
    elif Theta2_IK > 180:
        Theta2_IK -= 360
    
    # Tính toán theta3
    c23 = (Px_IK * math.cos(theta1_IK_rad) + Py_IK * math.sin(theta1_IK_rad) - L1 - L2 * math.cos(theta2_IK_rad) - L4 * math.cos(t_rad)) / L3
    s23 = (Pz_IK - D1 - H - L2 * math.sin(theta2_IK_rad) - L4 * math.sin(t_rad)) / L3
    theta3_IK_rad = math.atan2(s23, c23) - theta2_IK_rad
    Theta3_IK = math.degrees(theta3_IK_rad)
    Theta3_IK = round(Theta3_IK)
    
    if Theta3_IK < -180:
        Theta3_IK += 360
    elif Theta3_IK > 180:
        Theta3_IK -= 360
    
    # Tính toán theta4
    theta4_IK_rad = t_rad - theta2_IK_rad - theta3_IK_rad
    Theta4_IK = math.degrees(theta4_IK_rad)
    Theta4_IK = round(Theta4_IK)
    
    return Theta1_IK, Theta2_IK, Theta3_IK, Theta4_IK

# Forward Kinematics Function (unchanged)
def compute_fk(theta1, theta2, theta3, theta4):
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    theta3_rad = np.radians(theta3)
    theta4_rad = np.radians(theta4)
    
    x0, y0, z0 = 0, 0, H
    x1 = np.cos(theta1_rad) * L1
    y1 = np.sin(theta1_rad) * L1
    z1 = H + D1

    x2 = x1 + np.cos(theta1_rad) * L2 * np.cos(theta2_rad)
    y2 = y1 + np.sin(theta1_rad) * L2 * np.cos(theta2_rad)
    z2 = z1 + L2 * np.sin(theta2_rad)

    x3 = x2 + np.cos(theta1_rad) * L3 * np.cos(theta2_rad + theta3_rad)
    y3 = y2 + np.sin(theta1_rad) * L3 * np.cos(theta2_rad + theta3_rad)
    z3 = z2 + L3 * np.sin(theta2_rad + theta3_rad)

    x4 = x3 + np.cos(theta1_rad) * L4 * np.cos(theta2_rad + theta3_rad + theta4_rad)
    y4 = y3 + np.sin(theta1_rad) * L4 * np.cos(theta2_rad + theta3_rad + theta4_rad)
    z4 = z3 + L4 * np.sin(theta2_rad + theta3_rad + theta4_rad)

    return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2), (x3, y3, z3), (x4, y4, z4)]

# Example input for the end effector's position
Px_IK, Py_IK, Pz_IK, Theta_IK = 140, 20, 10, -90

# Compute inverse kinematics
Theta1_IK, Theta2_IK, Theta3_IK, Theta4_IK = inverse_kinematics(Px_IK, Py_IK, Pz_IK, Theta_IK)

# Print the calculated angles
print("Calculated joint angles from inverse kinematics:")
print(f"Theta1: {Theta1_IK}°")
print(f"Theta2: {Theta2_IK}°")
print(f"Theta3: {Theta3_IK}°")
print(f"Theta4: {Theta4_IK}°")

# Compute the joint positions from forward kinematics
positions = compute_fk(Theta1_IK, Theta2_IK, Theta3_IK, Theta4_IK)

# Plotting the robot arm in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = ['r', 'g', 'b', 'purple']  # Different colors for each link

# Plot each link with its respective color
for i in range(len(positions) - 1):
    x_values = [positions[i][0], positions[i + 1][0]]
    y_values = [positions[i][1], positions[i + 1][1]]
    z_values = [positions[i][2], positions[i + 1][2]]
    ax.plot(x_values, y_values, z_values, color=colors[i], linewidth=2)

# Show the coordinates of the end effector
end_effector = positions[-1]
ax.text(end_effector[0], end_effector[1], end_effector[2],
        f"({end_effector[0]:.2f}, {end_effector[1]:.2f}, {end_effector[2]:.2f})",
        color='black', fontsize=12, weight='bold')

# Set labels and title
ax.set_xlabel('Px')
ax.set_ylabel('Py')
ax.set_zlabel('Pz')
plt.show()
