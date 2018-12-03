from phlib import *
from integrated_pd_controller import *


def integrate(x_desired, y_desired, z_desired, interval, iterations):
    # Coordinates
    x = [0]
    y = [0]
    z = [0]

    # Velocities
    d1x = [0]
    d1y = [0]
    d1z = [0]

    # Accelerations
    d2x = [0]
    d2y = [0]
    d2z = [0]

    # Angles
    φ = [0]
    θ = [0]
    ψ = [0]

    # Angular velocities
    d1φ = [0]
    d1θ = [0]
    d1ψ = [0]

    # Angular accelerations
    d2φ = [0]
    d2θ = [0]
    d2ψ = [0]

    # Engines' velocities
    ω1 = [0]
    ω2 = [0]
    ω3 = [0]
    ω4 = [0]

    # Control angles and their derivatives
    φc = [0]
    θc = [0]
    ψc = [0]

    for i in range(1, iterations):
        # Find desired engines velocities
        ω1_d = ω1_desired(x[i - 1], d1x[i - 1], d2x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], d2y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], d2z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1], φc,
                          θ[i - 1], d1θ[i - 1], θc,
                          ψ[i - 1], d1ψ[i - 1], ψc,
                          interval)
        ω2_d = ω2_desired(x[i - 1], d1x[i - 1], d2x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], d2y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], d2z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1], φc,
                          θ[i - 1], d1θ[i - 1], θc,
                          ψ[i - 1], d1ψ[i - 1], ψc,
                          interval)
        ω3_d = ω3_desired(x[i - 1], d1x[i - 1], d2x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], d2y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], d2z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1], φc,
                          θ[i - 1], d1θ[i - 1], θc,
                          ψ[i - 1], d1ψ[i - 1], ψc,
                          interval)
        ω4_d = ω4_desired(x[i - 1], d1x[i - 1], d2x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], d2y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], d2z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1], φc,
                          θ[i - 1], d1θ[i - 1], θc,
                          ψ[i - 1], d1ψ[i - 1], ψc,
                          interval)

        # Find engines' velocities

        if type(ω1_d) is complex:
            ω1_d = 0
        if type(ω2_d) is complex:
            ω2_d = 0
        if type(ω3_d) is complex:
            ω3_d = 0
        if type(ω4_d) is complex:
            ω4_d = 0

        ω1.append(min(max(ω1_d, 0), 500))
        ω2.append(min(max(ω2_d, 0), 500))
        ω3.append(min(max(ω3_d, 0), 500))
        ω4.append(min(max(ω4_d, 0), 500))

        # Find angular accelerations
        d2φ.append(calc_d2φ(ω2[i], ω4[i]))
        d2θ.append(calc_d2θ(ω1[i], ω3[i]))
        d2ψ.append(calc_d2ψ(ω1[i], ω2[i], ω3[i], ω4[i]))

        # Find angular velocities
        d1φ.append(d1φ[i - 1] + d2φ[i] * interval)
        d1θ.append(d1θ[i - 1] + d2θ[i] * interval)
        d1ψ.append(d1ψ[i - 1] + d2ψ[i] * interval)

        # Find angles
        φ.append(φ[i - 1] + d1φ[i - 1] * interval + d2φ[i] * interval ** 2 / 2)
        θ.append(θ[i - 1] + d1θ[i - 1] * interval + d2θ[i] * interval ** 2 / 2)
        ψ.append(ψ[i - 1] + d1ψ[i - 1] * interval + d2ψ[i] * interval ** 2 / 2)

        # Find linear accelerations
        d2x.append(calc_d2x(ω1[i], ω2[i], ω3[i], ω4[i], φ[i], θ[i], ψ[i], d1x[i - 1]))
        d2y.append(calc_d2y(ω1[i], ω2[i], ω3[i], ω4[i], φ[i], θ[i], ψ[i], d1y[i - 1]))
        d2z.append(calc_d2z(ω1[i], ω2[i], ω3[i], ω4[i], φ[i], θ[i], ψ[i], d1z[i - 1]))

        # Find linear velocities
        d1x.append(d1x[i - 1] + d2x[i] * interval)
        d1y.append(d1y[i - 1] + d2y[i] * interval)
        d1z.append(d1z[i - 1] + d2z[i] * interval)

        # Find linear coordinates
        x.append(x[i - 1] + d1x[i - 1] * interval + d2x[i] * interval ** 2 / 2)
        y.append(y[i - 1] + d1y[i - 1] * interval + d2y[i] * interval ** 2 / 2)
        z.append(z[i - 1] + d1z[i - 1] * interval + d2z[i] * interval ** 2 / 2)

    print("ω1", ω1)
    print("ω2", ω2)
    print("ω3", ω3)
    print("ω4", ω4)

    print("d2φ", d2φ)
    print("d2θ", d2θ)
    print("d2ψ", d2ψ)

    print("d1φ", d1φ)
    print("d1θ", d1θ)
    print("d1ψ", d1ψ)

    print("φ", φ)
    print("θ", θ)
    print("ψ", ψ)

    print("d2x", d2x)
    print("d2y", d2y)
    print("d2z", d2z)

    print("d1x", d1x)
    print("d1y", d1y)
    print("d1z", d1z)

    print("x", x)
    print("y", y)
    print("z", z)

    return x, y, z
