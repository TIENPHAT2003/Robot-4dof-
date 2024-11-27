import numpy as np
import matplotlib.pyplot as plt

# Hàm động học thuận
def forward_kinematics(d1, h, L1, L2, L3, L4, t1, t2, t3, t4):
    x = np.cos(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) + 
                                  L3 * np.cos(np.radians(t2 + t3)) + 
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    y = np.sin(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) + 
                                  L3 * np.cos(np.radians(t2 + t3)) + 
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    z = d1 + h + L2 * np.sin(np.radians(t2)) + \
        L3 * np.sin(np.radians(t2 + t3)) + \
        L4 * np.sin(np.radians(t2 + t3 + t4))
    return x, y, z

# Hàm động học nghịch
def inverse_kinematics(d1, h, L1, L2, L3, L4, x, y, z, t):
    k = np.sqrt(x**2 + y**2)
    t1 = np.degrees(np.arctan2(y / k, x / k))

    E = x * np.cos(np.radians(t1)) + y * np.sin(np.radians(t1)) - L1 - L4 * np.cos(np.radians(t))
    F = z - d1 - h - L4 * np.sin(np.radians(t))

    a = -2 * L2 * F
    b = -2 * L2 * E
    d = L3**2 - E**2 - F**2 - L2**2
    e = np.sqrt(a**2 + b**2)

    alpha = np.degrees(np.arctan2(-2 * L2 * F / e, -2 * L2 * E / e))
    discriminant = 1 - d**2 / (e**2)

    if discriminant < 0:
        raise ValueError("Tọa độ vượt quá workspace, không thể tính động học nghịch")

    t2_1 = np.degrees(np.arctan2(np.sqrt(discriminant), d / e)) + alpha
    t2_2 = np.degrees(np.arctan2(-np.sqrt(discriminant), d / e)) + alpha

    C23_1 = (x * np.cos(np.radians(t1)) + y * np.sin(np.radians(t1)) - 
             L1 - L2 * np.cos(np.radians(t2_1)) - L4 * np.cos(np.radians(t))) / L3
    S23_1 = (z - d1 - h - L2 * np.sin(np.radians(t2_1)) - L4 * np.sin(np.radians(t))) / L3
    t3_1 = np.degrees(np.arctan2(S23_1, C23_1)) - t2_1

    C23_2 = (x * np.cos(np.radians(t1)) + y * np.sin(np.radians(t1)) - 
             L1 - L2 * np.cos(np.radians(t2_2)) - L4 * np.cos(np.radians(t))) / L3
    S23_2 = (z - d1 - h - L2 * np.sin(np.radians(t2_2)) - L4 * np.sin(np.radians(t))) / L3
    t3_2 = np.degrees(np.arctan2(S23_2, C23_2)) - t2_2

    t4_1 = t - t2_1 - t3_1
    t4_2 = t - t2_2 - t3_2

    return t1, (t2_1, t3_1, t4_1), (t2_2, t3_2, t4_2)

# Các thông số robot
H = 80  
D1 = 176
L1 = 91
L2 = 122
L3 = 78
L4 = 79

# Tọa độ đầu và cuối
start_point = [370, 0, 256]
end_point = [160, 50, 20]

# Tính góc khớp tại điểm đầu và cuối (dùng giải pháp bội nghiệm 1 cho cả 2 điểm)
theta_start_1 = inverse_kinematics(D1, H, L1, L2, L3, L4, *start_point, 0)[1]
theta_end_1 = inverse_kinematics(D1, H, L1, L2, L3, L4, *end_point, -90)[1]

theta_start_2 = inverse_kinematics(D1, H, L1, L2, L3, L4, *start_point, 0)[2]
theta_end_2 = inverse_kinematics(D1, H, L1, L2, L3, L4, *end_point, -90)[2]

# Quy hoạch quỹ đạo cho cả hai bội nghiệm
tf = 4  # thời gian kết thúc
t = np.arange(0, tf + 0.01, 0.01)

# Tính hệ số quỹ đạo cho mỗi bội nghiệm
a0_1 = np.array(theta_start_1)
a0_2 = np.array(theta_start_2)
a1_1 = np.zeros(3)
a1_2 = np.zeros(3)
a2_1 = 3 * (np.array(theta_end_1) - np.array(theta_start_1)) / tf**2
a2_2 = 3 * (np.array(theta_end_2) - np.array(theta_start_2)) / tf**2
a3_1 = -2 * (np.array(theta_end_1) - np.array(theta_start_1)) / tf**3
a3_2 = -2 * (np.array(theta_end_2) - np.array(theta_start_2)) / tf**3

# Tính quỹ đạo cho mỗi bội nghiệm
qt_1 = np.zeros((len(t), 4))
qt_2 = np.zeros((len(t), 4))
for i in range(3):
    qt_1[:, i] = a0_1[i] + a1_1[i] * t + a2_1[i] * t**2 + a3_1[i] * t**3
    qt_2[:, i] = a0_2[i] + a1_2[i] * t + a2_2[i] * t**2 + a3_2[i] * t**3

# Tính tọa độ (x, y, z) từ quỹ đạo cho mỗi bội nghiệm
x_traj_1, y_traj_1, z_traj_1 = [], [], []
x_traj_2, y_traj_2, z_traj_2 = [], [], []
for i in range(len(t)):
    x1, y1, z1 = forward_kinematics(D1, H, L1, L2, L3, L4, *qt_1[i, :])
    x2, y2, z2 = forward_kinematics(D1, H, L1, L2, L3, L4, *qt_2[i, :])
    x_traj_1.append(x1)
    y_traj_1.append(y1)
    z_traj_1.append(z1)
    x_traj_2.append(x2)
    y_traj_2.append(y2)
    z_traj_2.append(z2)

# Vẽ quỹ đạo 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Vẽ quỹ đạo cho mỗi bội nghiệm
ax.plot(x_traj_1, y_traj_1, z_traj_1, label='Trajectory 1', color='blue')
ax.plot(x_traj_2, y_traj_2, z_traj_2, label='Trajectory 2', color='red')

# Vẽ các điểm bắt đầu và kết thúc
ax.scatter(start_point[0], start_point[1], start_point[2], color="green", label="Điểm bắt đầu")
ax.scatter(end_point[0], end_point[1], end_point[2], color="red", label="Điểm kết thúc")

# Cài đặt tiêu đề và nhãn cho các trục
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Robot 4-DOF Trajectory')

# Thêm chú thích cho các điểm bắt đầu và kết thúc
ax.legend()

# Hiển thị đồ thị
plt.show()

