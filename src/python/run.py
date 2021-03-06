import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pd_controller import from_genom
from pd_integrator import integrate

K_FILE = "./../resources/k.json"

desired = {"x": 3, "y": -7, "z": 5}
interval = 0.1
iterations = 1000

if __name__ == '__main__':
    with open(K_FILE) as f:
        from_genom(json.load(f))

    x, y, z, φ, θ, ψ, _ = integrate(desired["x"], desired["y"], desired["z"], interval, iterations)

    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal', projection='3d')
    ax.plot(x, y, z, "g.-")
    ax.plot([x[0]], [y[0]], [z[0]], ".b")
    ax.plot([desired["x"]], [desired["y"]], [desired["z"]], ".r")
    ax.set_xlim3d(min(x + [-1]), max(x + [1]))
    ax.set_ylim3d(min(y + [-1]), max(y + [1]))
    ax.set_zlim3d(0, max(z + [1]))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.show()

    plt.plot(φ, "g")
    plt.plot(θ, "c")
    plt.plot(ψ, "m")
    plt.plot(0, 0, ".b")
    plt.xlabel("Time")
    plt.legend(["Крен φ", "Тангаж θ", "Рысканье ψ"])
    plt.show()

    plt.plot(x, "g")
    plt.plot(y, "c")
    plt.plot(z, "m")
    plt.plot(0, 0, ".b")
    plt.plot(len(x), desired["x"], ".g")
    plt.plot(len(x), desired["y"], ".c")
    plt.plot(len(x), desired["z"], ".m")
    plt.xlabel("Time")
    plt.legend(["X", "Y", "Z"])
    plt.show()
