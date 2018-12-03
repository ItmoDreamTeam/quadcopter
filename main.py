import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from integrated_pd_controller_integrator import integrate

x, y, z = integrate(x_desired=0,
                    y_desired=0,
                    z_desired=0,
                    ω0=500,
                    interval=0.1,
                    iterations=100)

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
