import integrator

# Coordinates to achieve by the end of the flight
x_desired = 0
y_desired = 0
z_desired = 0

# Time interval (step) -> 0, seconds
interval = 0.1

# Engines velocities
ω1 = [0, 340]
ω2 = [0, 340]
ω3 = [0, 340]
ω4 = [0, 340]

x, y, z = integrator.integrate(ω1, ω2, ω3, ω4, interval)
