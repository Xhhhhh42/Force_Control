import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("data.txt")

# 提取时间、位置、速度和加速度数据
time_slow_new = data[:, 0]
q_com = data[:, 1:8]
dq_slow_new = data[:, 8:15]
ddq_slow_new = data[:, 15:22]

dof = 7

# 使用累积和计算速度
computed_dq_slow_new = np.cumsum(ddq_slow_new * 0.001, axis=0)

# 绘制比较图
fig, ax = plt.subplots(4,2)
for i in range(dof):
    row = i // 2
    col = i % 2
    ax[row, col].plot(time_slow_new, dq_slow_new[:, i], '-b', label='Original Velocity')
    ax[row, col].plot(time_slow_new, computed_dq_slow_new[:, i], '--r', label='Integrated Acceleration')
    ax[row, col].set_ylabel(f"Joint {i+1}")
    ax[row, col].legend()
    ax[row, col].grid()

plt.tight_layout()
plt.show()


