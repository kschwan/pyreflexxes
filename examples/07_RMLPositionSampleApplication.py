from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import reflexxes


gen = reflexxes.PositionTrajectoryGenerator(
    3,               # number of degrees of freedom
    0.001,           # cycle time (in seconds)
    [300, 100, 300], # max. vel.
    [300, 200, 100], # max. acc.
    [400, 300, 200]  # max. jerk
)

# set initial values
gen.current_position = [100, 0, 50]
gen.current_velocity = [100, -220, -50]
gen.current_acceleration = [-150, 250, -50]

x = [gen.current_position]
dx = [gen.current_velocity]
ddx = [gen.current_acceleration]

# generate trajectory
for pos, vel, acc in gen.trajectory([-600, -200, -350], [50, -50, -200], 6.5):
    x.append(pos)
    dx.append(vel)
    ddx.append(acc)

# plot the data
t = np.linspace(0, len(x) * gen.cycle_time, len(x))
fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
ax1.plot(t, x)
ax1.set_ylabel('position')
ax1.legend('123', title='DOF #')
ax2.plot(t, dx)
ax2.set_ylabel('velocity')
ax2.legend('123', title='DOF #')
ax3.plot(t, ddx)
ax3.set_ylabel('acceleration')
ax3.legend('123', title='DOF #')
ax3.set_xlabel('time')
plt.show()
