from wpimath.units import inchesToMeters
from math import pi

NEO_FREE_SPEED = 5676  # in RPM
ARM_CPR = 7  # arm motor counts per revolution
NEO_CPR = 42  # rev neo counts per revolution
DRIVE_WHEEL_RADIUS = inchesToMeters(5)
WHEEL_CIRCUMFERENCE = pi * DRIVE_WHEEL_RADIUS * 2
LINEAR_SPEED = ((2 * pi) * 7.5 * (NEO_FREE_SPEED / 60)) / DRIVE_WHEEL_RADIUS  # in m/s
