import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import integrator

# Coordinates to achieve by the end of the flight
x_desired = 10
y_desired = 10
z_desired = 10

# Time interval (step) -> 0, seconds
interval = 0.1

x, y, z = integrator.integrate_control(x_desired, y_desired, z_desired, ω0=0, interval=interval)

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', projection='3d')

ax.plot(x, y, z, "g.-")
ax.plot([x[0]], [y[0]], [z[0]], ".r")

ax.set_xlim3d(min(x + [-1]), max(x + [1]))
ax.set_ylim3d(min(y + [-1]), max(y + [1]))
ax.set_zlim3d(0, max(z + [1]))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

fig.show()
