from physics_base import *
from math import sin
from math import cos
from math import asin
from math import atan

KxP = 1.85
KyP = 8.55
KzP = 1.85
KφP = 3
KθP = 3
KψP = 3

KxD = 0.75
KyD = 0.75
KzD = 0.75
KφD = 0.75
KθD = 0.75
KψD = 0.75

KxDD = 1
KyDD = 1
KzDD = 1


# Equations (26) (http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
def calc_φc(Dx, Dy, Dz, ψ):
    return asin((Dx * sin(ψ) - Dy * cos(ψ)) / (Dx ** 2 + Dy ** 2 + (Dz + g) ** 2))


def calc_θc(Dx, Dy, Dz, ψ):
    return atan((Dx * cos(ψ) + Dy * sin(ψ)) / (Dz + g))


def calc_Tc(Dx, Dy, Dz, φ, θ, ψ):
    return m * (
            Dx * (sin(θ) * cos(ψ) * cos(φ) + sin(ψ) * sin(φ)) +
            Dy * (sin(θ) * sin(ψ) * cos(φ) - cos(ψ) * sin(φ)) +
            (Dz + g) * cos(θ) * cos(φ)
    )


# Equations (30) (http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
def calc_τφc(φ, d1φ, φc, d1φc):
    return Jx * (KφP * (φc - φ) + KφD * (d1φc - d1φ))


def calc_τθc(θ, d1θ, θc, d1θc):
    return Jy * (KθP * (θc - θ) + KθD * (d1θc - d1θ))


def calc_τψc(ψ, d1ψ, ψc, d1ψc):
    return Jz * (KψP * (ψc - ψ) + KψD * (d1ψc - d1ψ))


# Equations (29) (http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
# Let desired velocity and acceleration be 0
def calc_Dx(x, d1x, d2x, x_desired):
    return KxP * (x_desired - x) - KxD * d1x - KxDD * d2x


def calc_Dy(y, d1y, d2y, y_desired):
    return KyP * (y_desired - y) - KyD * d1y - KyDD * d2y


def calc_Dz(z, d1z, d2z, z_desired):
    return KzP * (z_desired - z) - KzD * d1z - KzDD * d2z


# Equations (24) (http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
def ω_desired_params(x, d1x, d2x, x_desired,
                     y, d1y, d2y, y_desired,
                     z, d1z, d2z, z_desired,
                     φ, d1φ, φcs,
                     θ, d1θ, θcs,
                     ψ, d1ψ, ψcs,
                     interval):
    Dx = calc_Dx(x, d1x, d2x, x_desired)
    Dy = calc_Dy(y, d1y, d2y, y_desired)
    Dz = calc_Dz(z, d1z, d2z, z_desired)

    T = calc_Tc(Dx, Dy, Dz, φ, θ, ψ)

    φc = calc_φc(Dx, Dy, Dz, ψ)
    θc = calc_θc(Dx, Dy, Dz, ψ)
    ψc = ψ

    τφ = calc_τφc(φ, d1φ, φc, (φc - φcs[-1]) / interval)
    τθ = calc_τθc(θ, d1θ, θc, (θc - θcs[-1]) / interval)
    τψ = calc_τψc(ψ, d1ψ, ψc, (ψc - ψcs[-1]) / interval)
    φcs.append(φc)
    θcs.append(θc)
    ψcs.append(ψc)
    return T, τφ, τθ, τψ


def ω1_desired(x, d1x, d2x, x_desired,
               y, d1y, d2y, y_desired,
               z, d1z, d2z, z_desired,
               φ, d1φ, φcs,
               θ, d1θ, θcs,
               ψ, d1ψ, ψcs,
               interval):
    T, τφ, τθ, τψ = ω_desired_params(x, d1x, d2x, x_desired,
                                     y, d1y, d2y, y_desired,
                                     z, d1z, d2z, z_desired,
                                     φ, d1φ, φcs,
                                     θ, d1θ, θcs,
                                     ψ, d1ψ, ψcs,
                                     interval)
    A = T / (4 * k)
    B = τθ / (2 * k * l)
    C = τψ / (4 * b)
    return (A - B - C) ** 0.5


def ω2_desired(x, d1x, d2x, x_desired,
               y, d1y, d2y, y_desired,
               z, d1z, d2z, z_desired,
               φ, d1φ, φcs,
               θ, d1θ, θcs,
               ψ, d1ψ, ψcs,
               interval):
    T, τφ, τθ, τψ = ω_desired_params(x, d1x, d2x, x_desired,
                                     y, d1y, d2y, y_desired,
                                     z, d1z, d2z, z_desired,
                                     φ, d1φ, φcs,
                                     θ, d1θ, θcs,
                                     ψ, d1ψ, ψcs,
                                     interval)
    A = T / (4 * k)
    B = τφ / (2 * k * l)
    C = τψ / (4 * b)
    return (A - B + C) ** 0.5


def ω3_desired(x, d1x, d2x, x_desired,
               y, d1y, d2y, y_desired,
               z, d1z, d2z, z_desired,
               φ, d1φ, φcs,
               θ, d1θ, θcs,
               ψ, d1ψ, ψcs,
               interval):
    T, τφ, τθ, τψ = ω_desired_params(x, d1x, d2x, x_desired,
                                     y, d1y, d2y, y_desired,
                                     z, d1z, d2z, z_desired,
                                     φ, d1φ, φcs,
                                     θ, d1θ, θcs,
                                     ψ, d1ψ, ψcs,
                                     interval)
    A = T / (4 * k)
    B = τθ / (2 * k * l)
    C = τψ / (4 * b)
    return (A + B - C) ** 0.5


def ω4_desired(x, d1x, d2x, x_desired,
               y, d1y, d2y, y_desired,
               z, d1z, d2z, z_desired,
               φ, d1φ, φcs,
               θ, d1θ, θcs,
               ψ, d1ψ, ψcs,
               interval):
    T, τφ, τθ, τψ = ω_desired_params(x, d1x, d2x, x_desired,
                                     y, d1y, d2y, y_desired,
                                     z, d1z, d2z, z_desired,
                                     φ, d1φ, φcs,
                                     θ, d1θ, θcs,
                                     ψ, d1ψ, ψcs,
                                     interval)
    A = T / (4 * k)
    B = τφ / (2 * k * l)
    C = τψ / (4 * b)
    return (A + B + C) ** 0.5
