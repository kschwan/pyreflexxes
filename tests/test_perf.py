from __future__ import print_function
import timeit


code_to_test = """
import reflexxes

N = 100

gen = reflexxes.PositionTrajectoryGenerator(
    3,               # number of DOF
    0.001,           # cycle time [sec]
    [300, 100, 300], # max vel.
    [300, 200, 100], # max acc.
    [400, 300, 200]  # max jerk
)

for i in range(N):
    gen.current_position = [100, 0, 50]
    gen.current_velocity = [100, -220, -50]
    gen.current_acceleration = [-150, 250, -50]

    for pos, vel, acc in gen.trajectory([-600, -200, -350], [50, -50, -200], 6.5):
        pass
"""

time_elapsed = timeit.timeit(code_to_test, number=1)
print("Time elapsed: {0:.3f} s".format(time_elapsed))
