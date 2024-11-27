import numpy as np
import matplotlib.pyplot as plt

# Hàm động học thuận
def forward_kinematics(d1, h, L1, L2, L3, L4, t1, t2, t3, t4):
    """
    Tính toán động học thuận cho robot 4-DOF.
    """
    x = np.cos(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) + 
                                  L3 * np.cos(np.radians(t2 + t3)) + 
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    y = np.sin(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) + 
                                  L3 * np.cos(np.radians(t2 + t3)) + 
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    z = d1 + h + L2 * np.sin(np.radians(t2)) + \
        L3 * np.sin(np.radians(t2 + t3)) + \
        L4 * np.sin(np.radians(t2 + t3 + t4))
    t = t2 + t3 + t4
    return x, y, z, t

# Các thông số của robot
H = 80  
D1 = 176
L1 = 91
L2 = 122
L3 = 78
L4 = 79

# Định nghĩa các thông số quỹ đạo
theta_0 = np.array([0, 0, 0, 0])  # Vị trí ban đầu
theta_f = np.array([90, 60, 30, 25])  # Vị trí cuối
tf = 4  # Thời gian di chuyển
t = np.arange(0, tf + 0.01, 0.01)

# Tính toán hệ số
a0 = theta_0
a1 = np.zeros(4)
a2 = 3 * (theta_f - theta_0) / tf**2
a3 = -2 * (theta_f - theta_0) / tf**3

# Tính quỹ đạo góc khớp
qt = np.zeros((len(t), 4))
for i in range(4):
    qt[:, i] = a0[i] + a1[i] * t + a2[i] * t**2 + a3[i] * t**3

# Tính tọa độ quỹ đạo end-effector
x, y, z = [], [], []
for theta in qt:
    t1, t2, t3, t4 = theta
    x_end, y_end, z_end, _ = forward_kinematics(D1, H, L1, L2, L3, L4, t1, t2, t3, t4)
    x.append(x_end)
    y.append(y_end)
    z.append(z_end)

# In ra tọa độ điểm bắt đầu và điểm kết thúc
start_point = (x[0], y[0], z[0])
end_point = (x[-1], y[-1], z[-1])
print(f"Tọa độ điểm bắt đầu: X = {start_point[0]:.2f}, Y = {start_point[1]:.2f}, Z = {start_point[2]:.2f}")
print(f"Tọa độ điểm kết thúc: X = {end_point[0]:.2f}, Y = {end_point[1]:.2f}, Z = {end_point[2]:.2f}")

# Vẽ quỹ đạo 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label="Quỹ đạo End-Effector", color="b")
ax.scatter(x[0], y[0], z[0], color="g", label=f"Điểm bắt đầu ({start_point[0]:.2f}, {start_point[1]:.2f}, {start_point[2]:.2f})")
ax.scatter(x[-1], y[-1], z[-1], color="r", label=f"Điểm kết thúc ({end_point[0]:.2f}, {end_point[1]:.2f}, {end_point[2]:.2f})")

# Thêm nhãn cho các điểm bắt đầu và kết thúc
ax.text(x[0], y[0], z[0], f"({start_point[0]:.2f}, {start_point[1]:.2f}, {start_point[2]:.2f})", color="green")
ax.text(x[-1], y[-1], z[-1], f"({end_point[0]:.2f}, {end_point[1]:.2f}, {end_point[2]:.2f})", color="red")

# Định dạng đồ thị
ax.set_title("Quỹ đạo di chuyển của End-Effector")
ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")
ax.legend()
plt.grid(True)
plt.show()
