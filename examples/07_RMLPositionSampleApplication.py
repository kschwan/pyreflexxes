from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import reflexxes


gen = reflexxes.extra.PositionTrajectoryGenerator(
    number_of_dofs=3,
    cycle_time=0.001,
    max_velocity=[300, 100, 300],
    max_acceleration=[300, 200, 100],
    max_jerk=[400, 300, 200]
)

# set initial values
gen.current_position = [100, 0, 50]
gen.current_velocity = [100, -220, -50]
gen.current_acceleration = [-150, 250, -50]

x = [gen.current_position.tolist()]
dx = [gen.current_velocity.tolist()]
ddx = [gen.current_acceleration.tolist()]

# generate trajectory
for pos, vel, acc in gen.trajectory([-600, -200, -350], [50, -50, -200], 6.5):
    x.append(pos)
    dx.append(vel)
    ddx.append(acc)

# plot the data
def on_key_press(event):
    if event.key == 'escape':
        plt.close()

t = np.linspace(0, len(x) * gen.cycle_time, len(x))
fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
fig.canvas.mpl_connect('key_press_event', on_key_press)
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
