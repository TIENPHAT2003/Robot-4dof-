import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Thông số của robot
H = 80
D1 = 176
L1 = 91
L2 = 122
L3 = 78
L4 = 79
step = 12  # Bước cho mỗi vòng lặp

# Hàm động học thuận (FK) tính tọa độ (x, y, z)
def FK(d1, h, L1, L2, L3, L4, t1, t2, t3, t4):
    x = np.cos(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) +
                                  L3 * np.cos(np.radians(t2 + t3)) +
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    y = np.sin(np.radians(t1)) * (L1 + L2 * np.cos(np.radians(t2)) +
                                  L3 * np.cos(np.radians(t2 + t3)) +
                                  L4 * np.cos(np.radians(t2 + t3 + t4)))
    z = d1 + h + L2 * np.sin(np.radians(t2)) + L3 * np.sin(np.radians(t2 + t3)) + L4 * np.sin(np.radians(t2 + t3 + t4))
    
    t = t2 + t3 + t4
    return x, y, z, t

# Khởi tạo không gian làm việc
workspace_points = []

# Duyệt qua các giá trị của góc khớp và tính toán vị trí đầu cuối
for t1 in range(-90, 91, step):
    for t2 in range(-90, 91, step):
        for t3 in range(-90, 91, step):
            for t4 in range(-90, 91, step):
                x, y, z, t= FK(D1, H, L1, L2, L3, L4, t1, t2, t3, t4)
                # Chỉ thêm các điểm có x >= 0
                if x >= 0:
                    workspace_points.append((x, y, z))

# Tách các giá trị x, y, z
x_vals, y_vals, z_vals = zip(*workspace_points)

# Vẽ không gian làm việc
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(x_vals, y_vals, z_vals, c=z_vals, cmap='jet', marker='o')

# Thiết lập trục và màu sắc
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# plt.title('Không gian làm việc Robot 4 bậc tự do')
plt.show()
