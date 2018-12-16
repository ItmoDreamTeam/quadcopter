from math import cos
from math import sin
from math import copysign

# *** Physical constants ***
g = 9.81  # gravitational acceleration, m/s^2
ρ = 1.2  # air density, kg/m^3

# *** Quadcopter parameters ***
l = 0.3  # distance between central part and an engine, m
center_mass = 0.5  # mass of central part, kg
engine_mass = 0.05  # mass of one engine, kg
center_radius = 0.1  # radius of central part, m
Sx = 0.03  # quadcopter projection perpendicular to X
Sy = 0.03  # quadcopter projection perpendicular to Y
Sz = 0.03  # quadcopter projection perpendicular to Z

# *** Experimentally determined parameters ***
c = 0.4  # drag coefficient
k = 1.4851e-5  # k = lift force / ω^2
b = 0.7426e-6

# *** Calculated values ***

# Total mass of quadcopter
m = center_mass + 4 * engine_mass

# Moments of inertia
Jx = Jy = 2 / 5 * center_mass * center_radius ** 2 + 2 * l ** 2 * engine_mass
Jz = 2 / 5 * center_mass * center_radius ** 2 + 4 * l ** 2 * engine_mass


# Moments of lift force
def τφ(ω2, ω4):
    return l * k * (ω4 ** 2 - ω2 ** 2)


def τθ(ω1, ω3):
    return l * k * (ω3 ** 2 - ω1 ** 2)


def τψ(ω1, ω2, ω3, ω4):
    return b * (-ω1 ** 2 + ω2 ** 2 - ω3 ** 2 + ω4 ** 2)


# Lift force - sum of all 4 engines' lift forces
def lift_force(ω1, ω2, ω3, ω4):
    return k * (ω1 ** 2 + ω2 ** 2 + ω3 ** 2 + ω4 ** 2)


# Angular accelerations
def calc_d2φ(ω2, ω4):
    return τφ(ω2, ω4) / Jx


def calc_d2θ(ω1, ω3):
    return τθ(ω1, ω3) / Jy


def calc_d2ψ(ω1, ω2, ω3, ω4):
    return τψ(ω1, ω2, ω3, ω4) / Jz


# Linear accelerations
def calc_d2x(ω1, ω2, ω3, ω4, φ, θ, ψ, d1x):
    return lift_force(ω1, ω2, ω3, ω4) / m * (cos(φ) * sin(θ) * cos(ψ) + sin(φ) * sin(ψ)) - \
           copysign(c * Sx * ρ * d1x ** 2 / 2, d1x)


def calc_d2y(ω1, ω2, ω3, ω4, φ, θ, ψ, d1y):
    return lift_force(ω1, ω2, ω3, ω4) / m * (cos(φ) * sin(θ) * sin(ψ) - sin(φ) * cos(ψ)) - \
           copysign(c * Sy * ρ * d1y ** 2 / 2, d1y)


def calc_d2z(ω1, ω2, ω3, ω4, φ, θ, ψ, d1z):
    return lift_force(ω1, ω2, ω3, ω4) / m * (cos(φ) * cos(θ)) - g - copysign(c * Sz * ρ * d1z ** 2 / 2, d1z)
