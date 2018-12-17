from scipy.optimize import *

from main import *
from integrator.ipd_integrator import *


def test(genom):
    from_genom(genom)
    loss = integrate(desired["x"], desired["y"], desired["z"], interval, iterations)[-1]
    print("loss", loss)
    return loss


def train():
    result = differential_evolution(test, bounds=[(0.001, 1)] * 15, maxiter=100, popsize=5)
    print(result)
    with open(".." + K_FILE, "w") as f:
        json.dump(list(result["x"]), f)


if __name__ == '__main__':
    train()
