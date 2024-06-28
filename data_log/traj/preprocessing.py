import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

traj_num = 20

slow_factor = 6

dof = 7
dt = 0.01
q0 = np.loadtxt("trajectories/Trajectory_{}/gen_q0.txt".format(traj_num))
dq = np.loadtxt("trajectories/Trajectory_{}/gen_dq.txt".format(traj_num))
dq = dq.reshape(-1,dof)
ddq = np.loadtxt("trajectories/Trajectory_{}/gen_ddq.txt".format(traj_num))
ddq = ddq.reshape(-1,dof)
time = np.loadtxt("trajectories/Trajectory_{}/gen_time.txt".format(traj_num))
q = q0 + np.cumsum(dq * dt, axis=0) # interpolate -> 0 to slow_factor * T with resolution 0.001

dq_slow = dq / slow_factor
ddq_slow = ddq / (slow_factor**2.)
time_slow = time * slow_factor

time_slow_new = np.linspace(0, time_slow[-1], int(time_slow[-1]/0.001))

dq_splines = [CubicSpline(time_slow, dq_slow[:,i]) for i in range(7)]
ddq_splines = [CubicSpline(time_slow, ddq_slow[:,i]) for i in range(7)]

dq_slow_new = np.array([dq_splines[i](time_slow_new) for i in range(7)]).T
ddq_slow_new = np.array([ddq_splines[i](time_slow_new) for i in range(7)]).T

q_com = q0 + np.cumsum(dq_slow_new * 0.001, axis=0)
dq_com = dq[0] + np.cumsum(ddq_slow_new * 0.001, axis=0)

fig, ax = plt.subplots(4,2)

for i in range(dof):
    row = i // 2
    col = i % 2
    # ax[row, col].plot(q[:, i], '-r')
    # ax[row, col].plot(q[:, i], '-r')
    ax[row, col].plot(time_slow_new, dq_com[:, i], '-b')
    ax[row, col].plot(time_slow_new, dq_slow_new[:, i], '-r')
    # ax[row, col].plot(dq_slow[:, i], '-r')
    ax[row, col].set_ylabel("{}th joint".format(i+1))
    ax[row, col].grid()
    
#plt.tight_layout()
#plt.show()

data = np.concatenate([time_slow_new[:,None], q_com, dq_slow_new, ddq_slow_new], axis=1)
np.savetxt("data.txt", data)
