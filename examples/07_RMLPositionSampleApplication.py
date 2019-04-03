from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import reflexxes

NUMBER_OF_DOFS = 3
CYCLE_TIME_IN_SECONDS = 0.001

gen = reflexxes.PositionTrajectoryGenerator(
    NUMBER_OF_DOFS,
    CYCLE_TIME_IN_SECONDS,
    [300, 100, 300],  # max. vel.
    [300, 200, 100]  # max. acc.
)

# set initial values
gen.current_position = [100, 0, 50]
gen.current_velocity = [100, -220, -50]
gen.current_acceleration = [-150, 250, -50]

x = [gen.current_position]
dx = [gen.current_velocity]
ddx = [gen.current_acceleration]

for pos, vel, acc in gen.trajectory([-600, -200, -350], [50, -50, -200], 6.5):
    x.append(pos)
    dx.append(vel)
    ddx.append(acc)

t = np.linspace(0, len(x) * CYCLE_TIME_IN_SECONDS, len(x))
fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
ax1.plot(t, x)
ax1.set_ylabel('position')
ax2.plot(t, dx)
ax2.set_ylabel('velocity')
ax3.plot(t, ddx)
ax3.set_ylabel('acceleration')
ax3.set_xlabel('time')
plt.show()
