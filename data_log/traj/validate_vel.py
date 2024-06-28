import numpy as np
import matplotlib.pyplot as plt

traj_num = 3
dof = 7

data = np.loadtxt("data.txt")
q0 = np.loadtxt("trajectories/Trajectory_{}/gen_q0.txt".format(traj_num))
#dq = np.loadtxt("trajectories/Trajectory_{}/gen_dq.txt".format(traj_num))
#dq = dq.reshape(-1,dof)

# 提取时间、位置、速度和加速度数据
time_slow_new = data[:, 0]
q_com = data[:, 1:8]
dq_slow_new = data[:, 8:15]
ddq_slow_new = data[:, 15:22]

dof = 7

# 使用累积和计算速度
computed_q_slow_new = q0 + np.cumsum(dq_slow_new * 0.001, axis=0)

# 绘制比较图
fig, ax = plt.subplots(4,2)
for i in range(dof):
    row = i // 2
    col = i % 2
    ax[row, col].plot(time_slow_new, q_com[:, i], '-b', label='Original Position')
    ax[row, col].plot(time_slow_new, computed_q_slow_new[:, i], '--r', label='Integrated Velocity')
    ax[row, col].set_ylabel(f"Joint {i+1}")
    ax[row, col].legend()
    ax[row, col].grid()

plt.tight_layout()
plt.show()


