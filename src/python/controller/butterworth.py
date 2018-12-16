from physics_base import *
from math import atan

T0 = 2.9

tx = 1
ty = 1
tz = 1
tφ = 0.15
tθ = 0.15
tψ = 1
tω = 0.05


# Desired values
def lift_force_desired(z_desired, z, d1z, φ, θ, ψ):
    return m * (g + copysign(c * Sz * ρ * d1z ** 2 / 2, d1z) - c1z() * (z - z_desired) - c2z() * d1z) / \
           (cos(φ) * cos(θ))


def d2x_desired(x_desired, x, d1x):
    return -c1x() * (x - x_desired) - c2x() * d1x


def d2y_desired(y_desired, y, d1y):
    return -c1y() * (y - y_desired) - c2y() * d1y


def θ_desired(x_desired, x, d1x, d1z):
    return atan((d2x_desired(x_desired, x, d1x) + copysign(c * Sx * ρ * d1x ** 2 / 2, d1x)) /
                (g + copysign(c * Sz * ρ * d1z ** 2 / 2, d1z)))


def φ_desired(y_desired, y, d1y, d1z, θ):
    return -atan((d2y_desired(y_desired, y, d1y) + copysign(c * Sy * ρ * d1y ** 2 / 2, d1y)) * cos(θ) /
                 (g + copysign(c * Sz * ρ * d1z ** 2 / 2, d1z)))


def τφ_desired(y_desired, y, d1y, d1z, φ, d1φ, θ):
    return Jx * (-c1φ() * (φ - φ_desired(y_desired, y, d1y, d1z, θ)) - c2φ() * d1φ)


def τθ_desired(x_desired, x, d1x, d1z, θ, d1θ):
    return Jy * (-c1θ() * (θ - θ_desired(x_desired, x, d1x, d1z)) - c2θ() * d1θ)


def τψ_desired(ψ, d1ψ, ψ_desired=0):
    return Jz * (-c1ψ() * (ψ - ψ_desired) - c2ψ() * d1ψ)


def ω1_desired(x, d1x, x_desired,
               y, d1y, y_desired,
               z, d1z, z_desired,
               φ, d1φ,
               θ, d1θ,
               ψ, d1ψ):
    A = lift_force_desired(z_desired, z, d1z, φ, θ, ψ) / (4 * k)
    B = τθ_desired(x_desired, x, d1x, d1z, θ, d1θ) / (2 * k * l)
    C = τψ_desired(ψ, d1ψ) / (4 * b)
    return (A - B - C) ** 0.5


def ω2_desired(x, d1x, x_desired,
               y, d1y, y_desired,
               z, d1z, z_desired,
               φ, d1φ,
               θ, d1θ,
               ψ, d1ψ):
    A = lift_force_desired(z_desired, z, d1z, φ, θ, ψ) / (4 * k)
    B = τφ_desired(y_desired, y, d1y, d1z, φ, d1φ, θ) / (2 * k * l)
    C = τψ_desired(ψ, d1ψ) / (4 * b)
    return (A - B + C) ** 0.5


def ω3_desired(x, d1x, x_desired,
               y, d1y, y_desired,
               z, d1z, z_desired,
               φ, d1φ,
               θ, d1θ,
               ψ, d1ψ):
    A = lift_force_desired(z_desired, z, d1z, φ, θ, ψ) / (4 * k)
    B = τθ_desired(x_desired, x, d1x, d1z, θ, d1θ) / (2 * k * l)
    C = τψ_desired(ψ, d1ψ) / (4 * b)
    return (A + B - C) ** 0.5


def ω4_desired(x, d1x, x_desired,
               y, d1y, y_desired,
               z, d1z, z_desired,
               φ, d1φ,
               θ, d1θ,
               ψ, d1ψ):
    A = lift_force_desired(z_desired, z, d1z, φ, θ, ψ) / (4 * k)
    B = τφ_desired(y_desired, y, d1y, d1z, φ, d1φ, θ) / (2 * k * l)
    C = τψ_desired(ψ, d1ψ) / (4 * b)
    return (A + B + C) ** 0.5


# Butterworth coefficients
def c1x():
    return c1(tx)


def c2x():
    return c2(tx)


def c1y():
    return c1(ty)


def c2y():
    return c2(ty)


def c1z():
    return c1(tz)


def c2z():
    return c2(tz)


def c1φ():
    return c1(tφ)


def c2φ():
    return c2(tφ)


def c1θ():
    return c1(tθ)


def c2θ():
    return c2(tθ)


def c1ψ():
    return c1(tψ)


def c2ψ():
    return c2(tψ)


def c1ω():
    return c1(tω)


def c2ω():
    return c2(tω)


def c1(t):
    return (T0 / t) ** 2


def c2(t):
    return 2 ** 0.5 * (T0 / t)
