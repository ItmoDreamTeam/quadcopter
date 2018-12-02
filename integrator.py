from phlib import *
from butterworth import *


def integrate_control(x_desired, y_desired, z_desired, ω0, interval):
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
    ω1 = [ω0]
    ω2 = [ω0]
    ω3 = [ω0]
    ω4 = [ω0]

    # Engines' accelerations
    d1ω1 = [0]
    d1ω2 = [0]
    d1ω3 = [0]
    d1ω4 = [0]

    # Derivatives of engines' accelerations
    d2ω1 = [0]
    d2ω2 = [0]
    d2ω3 = [0]
    d2ω4 = [0]

    for i in range(1, 10):
        # Find desired engines velocities
        ω1_d = ω1_desired(x[i - 1], d1x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1],
                          θ[i - 1], d1θ[i - 1],
                          ψ[i - 1], d1ψ[i - 1])
        ω2_d = ω2_desired(x[i - 1], d1x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1],
                          θ[i - 1], d1θ[i - 1],
                          ψ[i - 1], d1ψ[i - 1])
        ω3_d = ω3_desired(x[i - 1], d1x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1],
                          θ[i - 1], d1θ[i - 1],
                          ψ[i - 1], d1ψ[i - 1])
        ω4_d = ω4_desired(x[i - 1], d1x[i - 1], x_desired,
                          y[i - 1], d1y[i - 1], y_desired,
                          z[i - 1], d1z[i - 1], z_desired,
                          φ[i - 1], d1φ[i - 1],
                          θ[i - 1], d1θ[i - 1],
                          ψ[i - 1], d1ψ[i - 1])

        # Find derivatives of engines' accelerations
        d2ω1.append(-c1ω() * (ω1[i - 1] - ω1_d) - c2ψ() * d1ω1[i - 1])
        d2ω2.append(-c1ω() * (ω2[i - 1] - ω2_d) - c2ψ() * d1ω2[i - 1])
        d2ω3.append(-c1ω() * (ω3[i - 1] - ω3_d) - c2ψ() * d1ω3[i - 1])
        d2ω4.append(-c1ω() * (ω4[i - 1] - ω4_d) - c2ψ() * d1ω4[i - 1])

        # Find engines' accelerations
        d1ω1.append(d1ω1[i - 1] + d2ω1[i] * interval)
        d1ω2.append(d1ω2[i - 1] + d2ω2[i] * interval)
        d1ω3.append(d1ω3[i - 1] + d2ω3[i] * interval)
        d1ω4.append(d1ω4[i - 1] + d2ω4[i] * interval)

        # Find engines' velocities
        ω1.append(ω1[i - 1] + d1ω1[i - 1] * interval + d2ω1[i] * interval ** 2 / 2)
        ω2.append(ω2[i - 1] + d1ω2[i - 1] * interval + d2ω2[i] * interval ** 2 / 2)
        ω3.append(ω3[i - 1] + d1ω3[i - 1] * interval + d2ω3[i] * interval ** 2 / 2)
        ω4.append(ω4[i - 1] + d1ω4[i - 1] * interval + d2ω4[i] * interval ** 2 / 2)

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


def integrate(ω1, ω2, ω3, ω4, interval):
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

    for i in range(1, len(ω1)):
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
