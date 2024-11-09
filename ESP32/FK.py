import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

H   =   80  
D1  =   176 
L1 	=	91
L2	=	122
L3	=	78
L4	=	79

def compute_fk(theta1, theta2, theta3, theta4):
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    theta3_rad = np.radians(theta3)
    theta4_rad = np.radians(theta4)
    
    # Tọa độ của từng khớp
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

# Góc nhập vào cho mỗi khớp (thay giá trị phù hợp)
theta1, theta2, theta3, theta4 = 0, 90, 0, 0

# Tính toán vị trí các khớp
positions = compute_fk(theta1, theta2, theta3, theta4)

# Vẽ
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = ['r', 'g', 'b', 'purple']  # Mỗi liên kết có màu riêng

# Vẽ từng liên kết với màu khác nhau
for i in range(len(positions) - 1):
    x_values = [positions[i][0], positions[i + 1][0]]
    y_values = [positions[i][1], positions[i + 1][1]]
    z_values = [positions[i][2], positions[i + 1][2]]
    ax.plot(x_values, y_values, z_values, color=colors[i], linewidth=2)

# Hiển thị tọa độ của điểm cuối
end_effector = positions[-1]
ax.text(end_effector[0], end_effector[1], end_effector[2],
        f"({end_effector[0]:.2f}, {end_effector[1]:.2f}, {end_effector[2]:.2f})",
        color='black', fontsize=12, weight='bold')

# Thiết lập nhãn và tiêu đề
ax.set_xlabel('Px')
ax.set_ylabel('Py')
ax.set_zlabel('Pz')
plt.title('3D Forward Kinematics with Different Colors for Each Link and End Effector Position')
plt.show()