from scipy.optimize import differential_evolution

from run import *


def validate(genom):
    from_genom(genom)
    loss = integrate(desired["x"], desired["y"], desired["z"], interval, iterations)[-1]
    print("loss", loss)
    return loss


def train():
    result = differential_evolution(validate, bounds=[(0.001, 1)] * 15, maxiter=100, popsize=5)
    print(result)
    with open(K_FILE, "w") as source:
        json.dump(list(result["x"]), source)


if __name__ == '__main__':
    train()
